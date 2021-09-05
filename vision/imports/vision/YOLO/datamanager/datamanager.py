''' datamanager.py
this file contains...

'''
# +++ IMPORTS
import os
import copy
from math import log10, ceil
import torch
import cv2
from tqdm import tqdm
from .bbox_util import bb_xywh_to_cs
from .aug_funcs import AUG_FUNCS
from .parse_data import mkdir


# +++ STATIC GLOBAL VARIABLES
_CUDA_DEVICE = 'cuda:0'

# +++ CLASS: DataManager
class DataManager:
    ''' data manager class
    this class manages the data from neural networks. it loads the data from a
    binary .pt file as saved by the DataParser class. it also performs active
    random data augmentation upon the calling of batches, as specified when
    initialized.

    --- args ---
    data_path : str
        path to the .pt file to load the data from.
    **data_aug (all floats, all defualts are 0)
    === data creation **data_aug ===
        all values here are the amount of data to be created *relative* to the
        amount of data already in the data set.
        + mosaics : adds mosaic data augmentation.
        + mixup : adds mixup data augmentation.
        + cutmix : adds cutmix data augmentation.
        + cutout : cutout data augmentation.
    === data manipulation **data_aug ===
        all values here are the odds that a single image in a batch will
        undergo that specific data augmentation.
        + hflip : horizontal flipping.
        + vflip : vertical flipping.
        + rot : random rotation by a multiple of 90 degrees.
    === nitty gritty **data_aug values ===
        + iou_threshs=(0.7,0.2) : the iou thresholds for data augmentation.
            when trimming, boxes will be dropped if the iou of the original and
            the portion inside the trim is greater than the first threshold
            value. when cropping, boxes will only be kept if the iou of the
            original and cropped box exceeds the second threshold value.
        + cut_range=(0.3,0.5) : the minimum and maximum cut sizes respectively,
            both relative to the image size, during cutout data augmentation.
    '''
    # +++ STATIC VARIABLES
    font = cv2.FONT_HERSHEY_SIMPLEX
    columns = ['names','originals','dims','images','labels']

    # +++ BUILT IN METHODS
    def __init__(self, data_path, CUDA=False, **data_aug):
        # +++ load the data
        self.data_path = os.path.realpath(data_path)
        self.data, self.classes, self.input_dim = torch.load(self.data_path)
        # +++ save the info about data augmentation
        self.device = _CUDA_DEVICE if CUDA else 'cpu'
        self.data_aug_dict = copy.deepcopy(data_aug)
        iou_threshs = data_aug.get('iou_threshs', (0.7,0.2))
        cut_range = data_aug.get('cut_range', (0.3, 0.5))
        self._make_aug = {
            'make_mosaics_aug': {
                'n': int(data_aug.get('mosaics', 0.0) * self.get_len(False)),
                'iou_thresh': iou_threshs[1]},
            'make_mixup_aug': {
                'n': int(data_aug.get('mixup', 0.0) * self.get_len(False))},
            'make_cutmix_aug': {
                'n': int(data_aug.get('cutmix', 0.0) * self.get_len(False)),
                'iou_threshs': iou_threshs,
                'cut_range': cut_range},
            'make_cutout_aug': {
                'n': int(data_aug.get('cutout', 0.0) * self.get_len(False)),
                'cut_range': cut_range,
                'iou_thresh': iou_threshs[0]}}
        self._aug = {
            'hflip_aug': data_aug.get('cutout', 0.0),
            'vflip_aug': data_aug.get('vflip', 0.0),
            'rand_rot_aug': data_aug.get('rot', 0.0)}

    def __repr__(self):
        data_file = os.path.relpath(self.data_path)
        return f'DataManager({data_file})'

    def __getitem__(self, index):
        if isinstance(index, int) and 0 <= index < 5:
            return list(self.data[:, index])
        if isinstance(index, str) and index in self.columns:
            i = self.columns.index(index)
            return list(self.data[:,i])
        raise IndexError(f'invalid data manager index ({index})')

    # +++ GETTING METHODS
    def get_names(self):
        ''' get names

        --- returns ---
        list[str] : the list of names of data points for the data, in order.
        '''
        return self['names']

    def get_dims(self):
        ''' get dimensions

        --- returns ---
        torch.FloatTensor : a tensor containing the x,y dimensions for each
            data point in the data, in order.
        '''
        return torch.stack(self['dims'], dim=0)

    def get_imgs(self):
        ''' get images

        --- returns ---
        torch.FloatTensor : a tensor containing the all the images in the data
            stacked together, in order. if augmented = True, then augmented
            images will be included as well (at the end).
        '''
        return torch.stack(self[3], dim=0)

    def get_lbls(self):
        ''' get images

        --- returns ---
        list[torch.FloatTensor] : a list of the tensors containing the all the
            labels for the images in the data, in order. if augmented = True,
            augmented labels will be included as well (at the end).
        '''
        return self[4]

    def get_len(self, augmented=True):
        ''' get the number of data points
        
        --- args ---
        augmented : bool, optional (defualt=True)
            if true, the amount of data that will be augmented will be included
            as data points.

        --- returns ---
        int : number of data points in the set. 
        '''
        out = self.data.shape[0]
        if augmented:
            for f in self._make_aug:
                out += self._make_aug[f]['n']
        return out

    def get_class(self, class_num):
        ''' get class method
        gets the name of the class associated with the number provided.

        --- args ---
        class_num : int
            the number of the class.
        --- returns ---
        str : the name of the class.
        '''
        return self.classes[int(class_num)]

    # +++ DATA MANAGER METHODS
    def get_data(self, augment=True):
        ''' get data method
        this method gets the data (images and labels) that are given to the
        data manager.

        --- args ---
        augment : bool, optional (default=True)
            should augmented data be included in the

        --- returns ---
        torch.FloatTensor : the augmented images.
        list[torch.FloatTensor] : the augmented labels.
        '''
        # +++ get the dataset's data
        imgs = self.get_imgs()
        lbls = self.get_lbls()
        # +++ if not augmenting, return here
        if not augment:
            return imgs, lbls
        # +++ get ready to do data augmentation wootwoot
        imgs = imgs.to(self.device)
        lbls = [lbl.to(self.device) for lbl in lbls]
        aug_imgs = torch.FloatTensor(0, 3, self.input_dim, self.input_dim)
        aug_imgs = aug_imgs.to(self.device)
        aug_lbls = []
        # +++ create the augmented data that needs creating
        for func_name in self._make_aug:
            # skip if no images are being made
            if self._make_aug[func_name]['n'] == 0:
                continue
            # augment data
            aimgs, albls = AUG_FUNCS[func_name](
                imgs, lbls, **self._make_aug[func_name])
            # add augmented data to the set
            aug_imgs = torch.cat((aug_imgs, aimgs), dim=0)
            aug_lbls += albls
            # delete the vars so cuda doesn't keep storing them
            del aimgs
            del albls
        # +++ add all the created data to the set
        imgs = torch.cat((imgs, aug_imgs), dim=0)
        lbls = lbls + aug_lbls
        # +++ augment the data already in the dataset
        _aug = copy.deepcopy(self._aug) # make a copy of the aug dict to use
        for func_name in _aug:
            # skip if nothing is being augmented
            if _aug[func_name] == 0:
                continue
            # randomly choose images to be augmented
            idxs = torch.where(
                torch.rand((self.get_len(),)) < _aug[func_name])[0]
            # feed that shit through the function to augment
            AUG_FUNCS[func_name](imgs, lbls, idxs=idxs)
        # +++ move the data to the cpu and return
        imgs = imgs.cpu()
        lbls = [lbl.cpu() for lbl in lbls]
        return imgs, lbls

    def _make_batch(self, imgs, lbls, idxs):
        ''' make batch method
        this method takes in the images, labels, and indexes of the data points
        to use in this batch, and outputs a tuple with the (batch_data,
        batch_labels) that are perscribed by the indicies.

        --- args ---
        imgs : torch.FloatTensor like (N, 3, input_dim, input_dim)
            the images in the dataset.
        lbls : list[torch.FloatTensor like (_, 5)]
            the list of labels of the dataset.
        idxs : list[int(s)] or torch.LongTensor like (batch_size,)
            the indexes of images in this batch.

        --- returns ---
        torch.FloatTensor : the batch_data, with shape (batch_size, 3,
            input_dim, input_dim).
        torch.tensor : the batch_labels, with shape (_, 6).
        '''
        # get the images to use
        batch_data = imgs[idxs]
        # construct the labels tensor
        batch_labels = torch.FloatTensor(0,6)
        for i, j in enumerate(idxs):
            lbl = lbls[j]
            img_i_col = torch.FloatTensor(lbl.size(0), 1).fill_(i)
            lbl = torch.cat((img_i_col, lbl), dim=1)
            batch_labels = torch.cat((batch_labels, lbl), dim=0)
        # return
        return (batch_data, batch_labels)

    def batches(self, batch_size=None, mini_batch_size=None, shuffle=True, 
    augmented=True):
        ''' batches method
        this method makes batches with the perscribed hyper parameters. note
        that if mini_batch_size is NOT set, this will just return a list of
        batches, where as is it IS set, it will return a list of lists of mini
        batches.

        --- args ---
        batch_size : int, optional (default=None)
            the size that the returned batches should be. if not set, all the
            data will be put into a single batch.
        mini_batch_size : int, optional (default=None)
            the size of mini batches. if not set, all the data in a batch will
            be put into a single mini batch. also note that this should be a
            factor of batch_size.
        shuffle : bool, optional (defualt=True)
            if true, the data will be shuffled prior to batching.
        augmented : bool, optional (defualt=True)
            if true, augmented data will be generated for this set of batches.

        --- returns ---
        list[list[tuple[torch.tensor, torch.tensor]]] : a list of the batches,
            where each batch is a list containg tuples of mini batch data, mini
            batch labels.
        OR IF mini_batch_size = None
        list[tuple[torch.tensor, torch.tensor]] : the list of the batches,
            where each batch is a tuple of (batch_data, batch_labels).
        '''
        # error check
        if (batch_size is None) and (not mini_batch_size is None):
            raise ValueError('cannot have None batch_size and non-None '
                f'mini_batch_size ({mini_batch_size})')
        if (not batch_size is None) and \
            (not mini_batch_size is None) and \
            (batch_size % mini_batch_size != 0):
            raise ValueError(f'batch size ({batch_size}) must be a clean '
                f' multiple of mini batch size ({mini_batch_size})')
        # set batch / mini batch size
        make_mini_bs = bool(mini_batch_size)
        if batch_size is None:
            batch_size = self.get_len(augmented=augmented)
        if mini_batch_size is None:
            mini_batch_size = batch_size
        # get the data
        imgs, lbls = self.get_data(augment=augmented)
        # get the indexes to use in order, based on shuffle
        if shuffle:
            idxs = torch.randperm(self.get_len())
        else:
            idxs = torch.arange(self.get_len())
        # now make the batches
        num_batches = self.get_len() // batch_size
        num_mini_batches = batch_size // mini_batch_size
        batches = []
        for i in range(num_batches):
            mini_batches = []
            for j in range(num_mini_batches):
                start_i = i*batch_size + j*mini_batch_size
                end_i = i*batch_size + (j+1)*mini_batch_size
                mini_batches.append(self._make_batch(imgs, lbls,
                    idxs[start_i : end_i]))
            # add the mini_batch (or batch) to the list
            if make_mini_bs:
                batches.append(mini_batches)
            else:
                batches.append(mini_batches[0])
        # return, and maybe collapse the minibatches
        return batches

    def write_boxes(self, boxes, images=None, img_labels=None, 
    colorfile='colors.pt', rel_txt_h=0.02):
        ''' write boxes method
        this method is used to write boxes onto images.

        --- args ---
        boxes : list[torch.FloatTensor] OR torch.FloatTensor
            the boxes to be written to each image. 
            =>  if it is a LIST, it should contain tensors with size (_, 5) 
                where the first dimension contains
                    [class#, xcenter, ycenter, width, height]
                where all measurements are normalized by image dimension.
            =>  if it is a TENSOR, it should have size (_, 8) where the first
                dimension contains:
                    [img#, xmin, ymin, xmax, ymax, boxconf, clsconf, class#]
                where all measurements are in pixels of the images, as provided.
        images : list[numpy.ndarray], optional (defualt=None)
            the set of images that are having boxes written onto them, formatted
            exactly how cv2 reads them. if these are not provided, the data 
            manager's *original* images will be used.
        img_labels : list[string], optional (defualt=None)
            if provided, each string will be placed in the top left corner of
            each image. these should be in the same order as the images.
        colorfile : string, optional (default='colors.pt')
            the path to the color data for the boxes.
        
        --- returns ---
        list[numpy.ndarray] : list of the images with boxes written.
        '''
        # +++ get the colors
        colorfile_path = os.path.join(os.path.realpath('.'), 'datamanager', colorfile)
        f = open(os.path.realpath(colorfile_path), 'rb')
        color_list = torch.load(f)
        f.close()

        # +++ load up and duplicate the images
        if images == None: 
            images = self['originals']
        images = copy.deepcopy(images)

        # +++ unpack all the boxes into a common format
        all_boxes = []
        if isinstance(boxes, (list, tuple)): # list of tensors
            # loop through each image
            for img, img_boxes in zip(images, boxes):
                img_boxes = img_boxes.clone()

                # if there are no boxes, just add an empty list
                if img_boxes.numel() == 0:
                    all_boxes.append(())
                    continue

                # convert xywh to x/y min and x/y max for all the boxes
                bb_xywh_to_cs(img_boxes[:,1:5])

                # convert measuremnets to pixels
                imy, imx, _ = img.shape 
                img_boxes[:,[1,3]] *= imx 
                img_boxes[:,[2,4]] *= imy

                # get the corners for each box
                c1s = [tuple(c1) for c1 in img_boxes[:,1:3].long().tolist()]
                c2s = [tuple(c2) for c2 in img_boxes[:,3:5].long().tolist()]
                # make the labels for each box
                lbls = [self.get_class(i) for i in img_boxes[:,0]]

                # get colors for each box
                colors = [color_list[int(i)] for i in img_boxes[:,0]]

                # add the final list of image boxes to all_boxes
                out_img_boxes = list(zip(lbls, c1s, c2s, colors))
                all_boxes.append(out_img_boxes)
        elif isinstance(boxes, torch.FloatTensor): # tensor
            for i in range(len(images)):
                # get the boxes for this img
                img_boxes = boxes[torch.where(boxes[:,0] == i)]
                
                # get the corners of the boxes
                c1s = [tuple(c1) for c1 in img_boxes[:,1:3].int().tolist()]
                c2s = [tuple(c2) for c2 in img_boxes[:,3:5].int().tolist()]

                # make the labels for each detection
                lbls = [f'{self.get_class(det[7])};{det[5]:.3f},{det[6]:.3f}' \
                    for det in img_boxes]
                
                # get the colors for each box
                colors = [color_list[int(c)] for c in img_boxes[:,7]]

                # add the formatted list of boxes for this img to all_boxes
                out_img_boxes = list(zip(lbls, c1s, c2s, colors))
                all_boxes.append(out_img_boxes)
        else:
            raise AttributeError(f'invalid type for boxes ({type(boxes).__name__})')

        # +++ write the boxes and labels onto the images
        for i, (img, img_boxes) in enumerate(zip(images, all_boxes)):
            # +++ calculate the font_scale and thickness to use
            imy = img.shape[0]
            txt_h = rel_txt_h * imy # desired text height
            real_txt_h =    [cv2.getTextSize(lbl, self.font, 1, 1)[0][1] \
                                for lbl, _, _, _ in img_boxes] + \
                            [cv2.getTextSize('abcd0123', self.font, 1, 1)[0][1]]
            font_scale = float(txt_h / max(real_txt_h))
            font_thickness = max(1, int(txt_h/10))
            box_thickness = int(font_thickness * 2)

            # +++ write the image label, if there is one
            if img_labels != None:
                lbl = img_labels[i]

                # get the text size
                txt_size = cv2.getTextSize(lbl, self.font, font_scale, font_thickness)[0]

                # get important points
                c1 = (0,0)
                c2 = (txt_size[0]+4, txt_size[1]+4)
                org = (2, txt_size[1]+2)

                # write that shit
                cv2.rectangle(img, c1, c2, [0,0,0], -1) # text box
                cv2.putText(img, lbl, org, self.font, font_scale, [255,255,255], font_thickness) # label
            
            # +++ check if there are any boxes to write
            if len(img_boxes) == 0:
                continue

            # +++ now loop through to write the boxes
            for lbl, c1, c2, color in img_boxes:
                # get the text size
                txt_size = cv2.getTextSize(lbl, self.font, font_scale, font_thickness)[0]

                # make the textbox corners
                ct1 = (c1[0], c1[1] - txt_size[1] - 4) # top left of txt box
                ct2 = (c1[0] + txt_size[0] + 4, c1[1]) # bottom right of txt box

                # now write shit to the image
                cv2.rectangle(img, c1, c2, color, box_thickness) # bounding box
                cv2.rectangle(img, ct1, ct2, color, -1) # text box
                cv2.putText(img, lbl, c1, self.font, font_scale, [255,255,255], font_thickness) # text
        return images

    def scale_detections(self, detections):
        ''' scale detections method
        this method takes in some detections that have been made on the rescaled
        images and scales them back to fit the original images. it also makes
        sure that no detections go outside of the image dimensions.

        --- args ---
        detections : torch.FloatTensor with size (num_detections, >=5)
            the tensor should store the detections as
            [[img_index, xmin, ymin, xmax, ymax, ...], ...]
            where the image index is absolute in terms of the dataset, and the
            measurements are in units of pixels of the resized images.

        --- returns ---
        torch.FloatTensor with same size : the scaled detections that can be mapped
            onto the original images of this set.
        '''
        # make a copy of the detections
        detections = detections.clone()
        # get the scaling factors for each detection
        img_dims = self.get_dims()[detections[:,0].long()]
        scl_facs = img_dims/self.input_dim
        # scale the detections
        detections[:,1:5] *= scl_facs.repeat(1,2)
        # make sure they are all in the valid range
        detections[:,1:5] = torch.clamp(detections[:,1:5], min=0)
        detections[:,1:5] = torch.min(detections[:,1:5], img_dims.repeat(1,2))
        return detections

    def premake_batches(self, output_directory, num_epochs, batch_size=None, 
    mini_batch_size=None, shuffle=True, augmented=True):
        ''' pre-make batches method
        this method makes a ton of epochs with the perscribed hyper params.
        these batches can then be loaded by the DataLoader class for later use
        in training. this cuts down a LOT on the training time, since most of
        the training is just data augmentation and this allows for that to
        happen before hand.

        note that this method will also write a file (data-config.txt) to the
        output directory containing the useful information about the data set,
        so that you don't have to remember what each parameter is.

        each epoch will be written to it's own file in the output directory,
        and it will contain a list of batches. see the batches method on what
        a batch will look like depending on the mini_batch_size and whatnot.

        --- args ---
        output_directory: str
            the local directory to output items to. will be made if it doesn't
            already exist.
        num_epochs : int
            the number of epochs worth of batches that will be written.
        batch_size : int, optional (default=None)
            the size that the returned batches should be. if not set, all the
            data will be put into a single batch.
        mini_batch_size : int, optional (default=None)
            the size of mini batches. if not set, all the data in a batch will
            be put into a single mini batch. also note that this should be a
            factor of batch_size.
        shuffle : bool, optional (defualt=True)
            if true, the data will be shuffled prior to batching.
        augmented : bool, optional (defualt=True)
            if true, augmented data will be generated for this set of batches.

        --- returns ---
        list[list[tuple[torch.tensor, torch.tensor]]] : a list of the batches,
            where each batch is a list containg tuples of mini batch data, mini
            batch labels.
        OR IF mini_batch_size = None
        list[tuple[torch.tensor, torch.tensor]] : the list of the batches,
            where each batch is a tuple of (batch_data, batch_labels).
        '''
        # +++ set things up
        out_dir = mkdir(output_directory)
        ndig = ceil(log10(num_epochs))

        # +++ save the data manager
        torch.save(self, 'DataManager.pt')

        # +++ make the file that says all the info and shit
        lines = [
            '=== CALL PARAMETERS ===\n',
            f'num_epochs = {num_epochs}\n',
            f'batch_size = {batch_size}\n',
            f'mini_batch_size = {mini_batch_size}\n',
            f'shuffle = {shuffle}\n',
            f'augmented = {augmented}',
            '\n=== GENERAL DATA INFO ===\n',
            f'data_path = {self.data_path}\n',
            f'input_dim = {self.input_dim}\n',
            f'num_epochs = {num_epochs}\n',
            f'num_epochs = {num_epochs}\n',
            f'num_epochs = {num_epochs}\n']
        # add classes
        lines.append('classes:\n')
        for i, c in enumerate(self.classes):
            lines.append(f'\t#{i} = {c}\n')
        # add data aug info
        lines.append('\n=== DATA AUGMENTATION INFO ===\n')
        for key in self._make_aug:
            lines.append(f'{key} : {self.data_aug_dict.get(key, 0.0)}\n')
        for key in self._aug:
            lines.append(f'{key} : {self.data_aug_dict.get(key, 0.0)}\n')
        lines.append(f"iou_threshs = {self.data_aug_dict.get('iou_threshs', (0.7,0.2))}")
        lines.append(f"cut_range = {self.data_aug_dict.get('cut_range', (0.3, 0.5))}")
        # save it
        txtf = open(os.path.join(out_dir, 'data-config.txt'), 'w')
        txtf.writelines(lines)
        txtf.close()

        # +++ main loop
        for i in tqdm(range(num_epochs)):
            batches = self.batches(
                batch_size=batch_size,
                mini_batch_size=mini_batch_size, 
                shuffle=shuffle, augmented=augmented)
            out_f = os.path.join(out_dir, f'epoch-{i:0>{ndig}}.pt')
            torch.save(batches, out_f)

class DataLoader:
    ''' DataLoader class
    loads data that is saved by DataManager.premake_batches.
    
    '''
    def __init__(self, path):
        # load the relevant info from the path
        self.path = os.path.realpath(path)
        self.file_paths = list(filter(
            lambda x : x.split('.')[-1] == 'pt', os.listdir(self.path)))
        self.file_paths.sort()
        self.file_paths = [os.path.join(self.path, f) for f in self.file_paths]
        # setup the iterator parameters
        self.i = 0
        self.l = len(self.file_paths)

    # +++ basic methods
    def __repr__(self):
        ''' string representation of data loader '''
        return f'DataLoader({self.path})'

    def __len__(self):
        ''' length of data loader (total # of batches)'''
        return self.l
    
    # +++ data loader methods
    def batches(self):
        ''' batches
        returns the next batch in the data loader.
        '''
        batch = torch.load(self.file_paths[self.i])
        self.i = (self.i + 1) % self.l
        return batch
