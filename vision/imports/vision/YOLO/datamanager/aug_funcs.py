''' dataaug.py
this file contains functions that are very useful in augmenting data for the
DataManager class.
'''
# +++ IMPORTS
import torch
from .bbox_util import bb_xywh_to_cs, bb_cs_to_xywh, bb_cs_iou

# +++ STATIC GLOBAL VARIABLES
_CUDA_DEVICE = 'cuda:0'

# +++ UTILITY / INTERNAL FUNCTIONS
def crop_datas(imgs, lbls, crops, iou_thresh=0.2):
    ''' crop data function
    this function crops image and label data to be within a certain specified
    box. it will crop out label data if the iou of the cropped bounding box
    with the previous bounding box is less than the specified iou threshold.

    --- args ---
    imgs : torch.FloatTensor like (N, 3, y, x)
        the stack of image to crop.
    lbls : list[torch.FloatTensor like (D, 5)]
        the labels to crop in class,x,y,w,h format.
    crops : torch.FloatTensor like (N, 4)
        the places to crop each image, in xmin, ymin, xmax, ymax format, where
        all values are in PIXELS.
    iou_thresh : float, optional (default=0.2)
        a label will be dropped if the IoU of the new box with the old box is
        below this threshold.
    
    --- returns ---
    list[torch.FloatTensor like (3, y', x')] : the cropped images.
    list[torch.FloatTensor like (D', 5)] : the cropped labels.
    '''
    # get input info
    device = imgs.device
    # loop to crop the datas
    new_imgs, new_lbls = [], []
    for img, lbl, crop in zip(imgs, lbls, crops):
        # +++ crop the image
        new_imgs.append(img[:, crop[1]:crop[3], crop[0]:crop[2]].clone())
        # +++ get to work cropping the label
        old_lbl = lbl.clone()
        bb_xywh_to_cs(old_lbl[:,1:])
        new_lbl = old_lbl.clone()
        # get the NORMALIZED corners of the cropping
        img_dims = torch.FloatTensor([img.size(2), img.size(1)])
        crop_c1 = (crop[:2] / img_dims).to(device)
        crop_c2 = (crop[2:] / img_dims).to(device)
        # crop the labels to be within the box
        new_lbl[:,1:3] = torch.min(torch.max(crop_c1, new_lbl[:,1:3]), crop_c2)
        new_lbl[:,3:5] = torch.min(torch.max(crop_c1, new_lbl[:,3:5]), crop_c2)
        # void out boxes with ious below the threshold
        valid_bbs = bb_cs_iou(new_lbl[:,1:], old_lbl[:,1:]) > iou_thresh
        new_lbl = new_lbl[valid_bbs]
        # normalize the new label to the cropped image
        new_lbl[:, 1:] -= crop_c1.repeat(2)
        new_lbl[:, 1:] /= (crop_c2 - crop_c1).repeat(2)
        # put the new label back into xywh
        new_lbl[:,1:] = bb_cs_to_xywh(new_lbl[:,1:])
        # add to output
        new_lbls.append(new_lbl)
    # return
    return new_imgs, new_lbls

def trim_datas(imgs, lbls, trims, iou_thresh=0.7):
    ''' trim data function
    this function trims image and label data to exclude a certain specified
    box. it will trim out label data if the iou of the portion of the bounding
    box that lies within the trim and the whole bounding box exceeds the iou
    threshold value.

    --- args ---
    imgs : torch.FloatTensor like (N, 3, y, x)
        the stack of images to crop.
    lbls : list[torch.FloatTensor like (D, 5)]
        the labels to crop in class,x,y,w,h format.
    trims : torch.FloatTensor like (N, 4)
        the box bounding the desired places the images should be trimmed, in
        xmin, ymin, xmax, ymax format, where all values are in PIXELS.
    iou_thresh : float
        the theshold iou as mentioned in the docstring.

    --- returns ---
    list[torch.FloatTensor like (3, y', x')] : the images with the trimmed box
        blacked out.
    list[torch.FloatTensor like (D', 5)] : the trimmed labels.
    '''
    # get device
    device = imgs.device
    # +++ loop to trim images and labels
    new_imgs, new_lbls = [], []
    for img, lbl, trim in zip(imgs, lbls, trims):
        # +++ black out the trimmed box in the image
        new_img = img.clone()
        new_img[:, trim[1]:trim[3], trim[0]:trim[2]] = 0
        new_imgs.append(new_img)
        # +++ trim the label
        # convert label to corners format and make another copy
        new_lbl = lbl.clone()
        bb_xywh_to_cs(new_lbl[:,1:])
        intrim_lbl = new_lbl.clone()
        # get the NORMALIZED corners of the trimming
        img_dims = torch.FloatTensor([img.size(2), img.size(1)])
        trim_c1 = (trim[:2] / img_dims).to(device)
        trim_c2 = (trim[2:] / img_dims).to(device)
        # crop the intrim labels to be within the box
        intrim_lbl[:,1:3] = torch.min(torch.max(trim_c1, intrim_lbl[:,1:3]), trim_c2)
        intrim_lbl[:,3:5] = torch.min(torch.max(trim_c1, intrim_lbl[:,3:5]), trim_c2)
        # remove boxes with ious above the threshold
        valid_bbs = bb_cs_iou(intrim_lbl[:,1:], new_lbl[:,1:]) < iou_thresh
        new_lbl = new_lbl[valid_bbs]
        # put the label back into xywh
        bb_cs_to_xywh(new_lbl[:,1:])
        # add to output
        new_lbls.append(new_lbl)
    # return
    return new_imgs, new_lbls

# +++ DATA CREATION FUNCTIONS
def make_mosaics_aug(imgs, lbls, n, iou_thresh=0.2):
    ''' make mosaics function
    makes n mosaics using the images and labels from a dataset.

    --- args ---
    imgs : torch.FloatTensor like (N, 3, D, D)
        the stack of images to be made into mosaics
    lbls : list[torch.FloatTensor like (_, 5)] with length N
        the list of labels that are associated with the images.
    n : int
        the number of mosaic images to return.
    iou_thresh : float, optional (default=0.2)
        the iou threshold for bounding boxes when cropping the image. if the
        iou of the cropped bounding box with the original has an iou < this
        threshold, it will be ommited from the final labels.

    --- returns ---
    torch.FloatTensor like (n, 3, D, D) : the mosaic images.
    list[torch.FloatTensor like (_, 5)] : the labels for the images.
    '''
    # retrieve basic info from the dataset
    dim = imgs.size(-1)
    n_data = imgs.size(0)
    device = imgs.device
    # +++ make grids of image indices to use
    midxs = torch.randint(0, n_data, (n, 2, 2))
    # make the mosaics of images using the grid indecies
    mosaic_imgs = imgs[midxs]
    mosaic_imgs = torch.cat((mosaic_imgs[:,0], mosaic_imgs[:,1]), dim=3)
    mosaic_imgs = torch.cat((mosaic_imgs[:,0], mosaic_imgs[:,1]), dim=3)
    # +++ now make the labels for the mosaics
    mosaic_lbls = []
    # scales and buffs to apply to the labels
    scale, right_buff, down_buff = torch.zeros((3,1,5)).to(device)
    scale[0,0], scale[0,1:] = 1, 0.5
    right_buff[0,1] = 1
    down_buff[0,2] = 1
    # loop to create and correctly scale labels
    for idxs in midxs:
        (i1, i2), (i3, i4) = idxs
        mosaic_lbls.append(torch.cat((
            lbls[i1],
            lbls[i2] + right_buff,
            lbls[i3] + down_buff,
            lbls[i4] + right_buff + down_buff), dim=0) * scale)
    # use random crops to get all the images back to the original dimension
    crops = torch.randint(0, dim, (n, 2)).repeat(1,2)
    crops[:,2:] += dim
    out = crop_datas(mosaic_imgs, mosaic_lbls, crops, iou_thresh=iou_thresh)
    # make the images back into a tensor
    return torch.stack(out[0], dim=0), out[1]

def make_mixup_aug(imgs, lbls, n, abs_norm=True):
    ''' mixup data augmentation

    --- args ---
    imgs : torch.FloatTensor
        the stack of images to mixup.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    n : int
        the number of mixup images to make.
    abs_norm : bool, optional (defualt=True)
        if true, the images will be normalized absolutely (i.e. minimum will be
        made 0, maximum will be 1) before they are merged. i believe this may
        help when mixing images with drastically different lighting.

    --- returns ---
    torch.FloatTensor : the mixup images
    list[torch.FloatTensor] : the labels for the mixup images.
    '''
    # +++ make the list of indexes for the first and second set of images
    nimgs = imgs.size(0)
    idxs1 = torch.randint(0, nimgs, (n,))
    idxs2 = torch.randint(0, nimgs, (n,))
    # +++ make the output images
    imgs1, imgs2 = imgs[idxs1], imgs[idxs2]
    # normalize the images (this helps with mixing relatively dark/light images)
    if abs_norm:
        imgs1 -= imgs1.min()
        imgs2 -= imgs2.min()
        imgs1 /= imgs1.max()
        imgs2 /= imgs2.max()
    # average the images
    out_imgs = (imgs[idxs1] + imgs[idxs2])/2
    # concatenate the labels
    out_lbls = [torch.cat((lbls[i], lbls[j]), dim=0) for i, j in zip(idxs1, idxs2)]
    return out_imgs, out_lbls

def make_cutmix_aug(imgs, lbls, n, cut_range=(0.3,0.5), iou_threshs=(0.7,0.2)):
    ''' make cutmix data
    this function makes cutmix data.

    --- args ---
    imgs : torch.FloatTensor
        the stack of images to use.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    n : int
        the number of cutmix images to make.
    cut_range : tuple[float,float], optional (default=(0.3,0.5))
        a tuple containing the minimum amd maximum (respectively) size of the
        portion of each image to be cut, relative to the original image size.
    iou_threshs : tuple[float,float], optional (default=(0.7,0.2))
        a tuple containing the trim and crop iou thresholds, respectively. when
        trimming images, labels will be excluded if the iou of the original box
        with the portion inside the trim exceeds the trim threshold. when
        cropping images, labels will be excluded if the iou of the original box
        with the portion inside the crop is below this threshold.

    --- returns ---
    torch.FloatTensor : the cutmix images
    list[torch.FloatTensor] : the labels for the cutmix images.
    '''
    # get basic info / unpack inputs
    nimgs = imgs.size(0)
    dim = imgs.size(-1)
    cut_range = [int(dim * x) for x in cut_range]
    device = imgs.device
    # +++ decide the cuts to make to the images
    cut_sizes = torch.randint(*cut_range, (n,2))
    trims = (torch.rand((n, 2)) * (dim - cut_sizes)).long().repeat(1,2)
    trims[:, 2:] += cut_sizes
    crops = (torch.rand((n, 2)) * (dim - cut_sizes)).long().repeat(1,2)
    crops[:, 2:] += cut_sizes
    # +++ trim and crop the images to get the pieces to put together
    idxs1, idxs2 = torch.randint(0, nimgs, (2,n))
    lbls1, lbls2 = [lbls[i] for i in idxs1], [lbls[i] for i in idxs2]
    imgs1, lbls1 = trim_datas(imgs[idxs1], lbls1, trims, iou_thresh=iou_threshs[0])
    imgs2, lbls2 = crop_datas(imgs[idxs2], lbls2, crops, iou_thresh=iou_threshs[1])
    # +++ put the images together
    out_imgs = []
    for img1, img2, trim in zip(imgs1, imgs2, trims):
        img1[:,trim[1]:trim[3], trim[0]:trim[2]] = img2
        out_imgs.append(img1)
    # +++ put the labels together
    cut_sizes, trims = cut_sizes.to(device), trims.to(device)
    out_lbls = []
    for lbl1, lbl2, trim, cut_size in zip(lbls1, lbls2, trims, cut_sizes):
        # put the second label into pixels
        lbl2[:,1:3] *= cut_size
        lbl2[:,3:5] *= cut_size
        # offset by the top left corner of the trim
        lbl2[:,1:3] += trim[0:2]
        # normalize by the final image dimensions
        lbl2[:,1:] /= dim
        # add to output
        out_lbls.append(torch.cat((lbl1,lbl2), dim=0))
    # make output images a tensor and return
    return torch.stack(out_imgs, dim=0), out_lbls

def make_cutout_aug(imgs, lbls, n, cut_range=(0.3,0.5), iou_thresh=0.7):
    ''' make cutout data augmentation
    this function makes cutout augmentad data using the images in the dataset.

    --- args ---
    imgs : torch.FloatTensor like (N, 3, D, D)
        the stack of images to have data cut out.
    lbls : list[torch.FloatTensor like (_, 5)] with length N
        the list of labels that are associated with the images.
    n : int
        the number of new images to make
    cut_range : tuple[float,float], optional (default=(0.3,0.5))
        a tuple containing the minimum amd maximum (respectively) size of the
        portion of each image to be cut, relative to the original image size.
    iou_thresh : float
        labels will be excluded if the iou of the original box with the portion
        inside the trim exceeds this threshold.
    '''
    # get / unpack the inputs
    nimgs = imgs.size(0)
    dim = imgs.size(-1)
    cut_range = [int(dim * x) for x in cut_range]
    # decide on what cuts to make for each image
    cut_sizes = torch.randint(*cut_range, (n, 2))
    trims = (torch.rand((n, 2)) * (dim - cut_sizes)).long().repeat(1,2)
    trims[:, 2:] += cut_sizes
    # pick which data points to trim
    idxs = torch.randint(0, nimgs, (n,))
    # trim the images
    out = trim_datas(imgs[idxs], [lbls[i] for i in idxs],
        trims, iou_thresh=iou_thresh)
    return torch.stack(out[0], dim=0), out[1]

# +++ DATA MANIPULATION FUNCTIONS
def hflip_aug(batch, lbls, idxs=None):
    ''' *in place* horizontal flip data augmentation
    flips images and their labels horizontally.
    
    --- args ---
    batch : torch.FloatTensor
        the batch of images to flip.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    idxs : list[int], optional (default=None)
        the indexes of the specific images to flip. if not specified, all
        images will be flipped.
    '''
    # set the indexes
    if idxs is None:
        idxs = list(range(len(lbls)))
    # flip them images!
    batch[idxs] = batch[idxs].flip(3)
    # flip the labels
    for i in idxs:
        lbls[i][:,1] = 1 - lbls[i][:,1]

def vflip_aug(batch, lbls, idxs=None):
    ''' *in place* vertical flip data augmentation
    flips images and their labels vertical.
    
    --- args ---
    batch : torch.FloatTensor
        the batch of images to flip.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    idxs : list[int], optional (default=None)
        the indexes of the specific images to flip. if not specified, all
        images will be flipped.
    '''
    # set the indexes
    if idxs is None:
        idxs = list(range(len(lbls)))
    # flip them images!
    batch[idxs] = batch[idxs].flip(2)
    # flip the labels
    for i in idxs:
        lbls[i][:,2] = 1 - lbls[i][:,2]

def rot90_aug(batch, lbls, idxs=None):
    ''' *in place* 90 degree rotation data augmentation
    rotates images and labels counter clockwise by 90 degrees.

    --- args ---
    batch : torch.FloatTensor
        the batch of images to rotate.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    idxs : list[int], optional (default=None)
        the indexes of the specific images to rotate. if not specified, all
        images will be rotated.
    '''
    # set the indexes
    if idxs is None:
        idxs = list(range(len(lbls)))
    # rotate the images
    batch[idxs] = torch.rot90(batch[idxs], 1, [2,3])
    # rotate the labels
    for i in idxs:
        x, y, w, h = lbls[i][:,1:].clone().t()
        lbls[i][:,1] = y
        lbls[i][:,2] = 1 - x
        lbls[i][:,3] = h
        lbls[i][:,4] = w

def rand_rot_aug(batch, lbls, idxs=None):
    ''' *in place* random rotation data augmentation
    rotates images and labels counter clockwise by some multiple of 90 degrees,
    but never 360 degrees.

    --- args ---
    batch : torch.FloatTensor
        the batch of images to rotate.
    lbls : list[torch.FloatTensor]
        the list of labels for the images.
    idxs : list[int], optional (default=None)
        the indexes of the specific images to rotate. if not specified, all
        images will be rotated.
    '''
    # make the indexes a tensor
    if idxs is None:
        idxst = torch.arange(batch.size(0))
    else:
        idxst = torch.LongTensor(idxs)
    # for each index, decide a number of times to be rotated
    nrots = torch.randint(1,4, idxst.size())
    # rotate shit!
    for i in range(3):
        rot90_aug(batch, lbls, idxs=idxst[nrots > i])

# +++ function dictionary
AUG_FUNCS = {
    'make_mosaics_aug': make_mosaics_aug,
    'make_mixup_aug': make_mixup_aug,
    'make_cutmix_aug': make_cutmix_aug,
    'make_cutout_aug': make_cutout_aug,
    'hflip_aug': hflip_aug,
    'vflip_aug': vflip_aug,
    'rot90_aug': rot90_aug,
    'rand_rot_aug': rand_rot_aug
}
