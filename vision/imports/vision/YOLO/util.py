''' util.py
this file contains a bunch of functions that are used throughout this project, such
as functions for image pre-processing, image post-processing, file reading, tensor
transformations, etc.

last updated 01/09/2021
'''
# +++ imports

import os
import cv2
import torch
import numpy as np

# +++ functions

def make_tup(val, tup_len=2, expand_method=0):
    '''
    a helpful little function to make arbitrary values into tuples, for instance
    an input of x will be returned as a tuple (x,x), or if tup_len=4, (x,x,x,x).
    '''
    # make sure it's the right tuple length
    assert tup_len in [2,4], f'tup_len must be 2 or 4, not {tup_len}'

    if isinstance(val, int): # if it's an int
        return (val,) * tup_len # return it as tuple w/ right length
    elif isinstance(val, (tuple, list)): # if it's a tuple or list
        if len(val) == tup_len: # if it already has the right length
            return val # return it as is
        elif len(val) * 2 == tup_len: # if it's half the right length
            if expand_method == 0: # if we should duplicate
                return val * 2 # double it and return
            elif expand_method == 1: # if we should expand
                a, b = val # split up the tup
                return (a,a,b,b) # return the expanded tuple
            else: # otherwise (error)
                raise ValueError(f'unknown expand method {expand_method}')
        else: # otherwise (error)
            raise AttributeError(f'cannot convert a tuple with length {len(val)} ' + \
                f'to a tuple of length {tup_len}')
    else: # otherwise (type error)
        raise TypeError(f'cannot call make_tup on argument val with type {type(val).__name__}')

def get_image_paths(image_dir):
    '''
    returns a list of the absolute paths to each image in a specified image
    directory.
    
    ---args---
    image_dir : str
        the directory to load images from
    
    ---returns---
    list[str(s)] : the absolute paths to each image in the directory
    '''
    image_paths = [] # list of the image paths
    current_path = os.path.realpath('.') # current path we are in
    files_in_folder = os.listdir(image_dir) # get all the files in the image directory

    ### go through files in the folder
    for filename in files_in_folder:
        file_ext = os.path.splitext(filename)[1] # get the file extention
        if file_ext in ['.jpeg', '.jpg', '.png']: # valid image files
            image_paths.append(os.path.join(current_path, image_dir, filename))
        else: # not a valid image file
            # just print a warning and ignore the file
            print(f'warning: ignoring non-image file in image directory {filename}')
    
    return image_paths

def image_resize(img, in_dim, default=128, device='cpu'):
    '''
    resizes image to (dim,dim) without changing aspect ratio.
    
    ---args---
    img : cv2.Image
        the image to resize
    in_dim : tuple[int, int]
        the (y, x) dimensions that the output tensor should be
    default : int between 0 and 255, optional (default=128)
        the defualt pixel brightness value
    
    ---returns---
    torch.tensor : the tensor representation of the resized image
    '''
    imy, imx = img.shape[0], img.shape[1] # y, x dimensions of the image
    outy, outx = in_dim
    scale_factor = min(outy/imy, outx/imx) # calculate the scale factor
    newy = int(scale_factor * imy) # calculate the new image dimensions
    newx = int(scale_factor * imx)
    resized_image = cv2.resize(img, (newx, newy), interpolation=cv2.INTER_CUBIC)
        # make the resized image
    ybuff = (outy - newy) // 2 # y dimension buffer
    xbuff = (outx - newx) // 2 # x dimension buffer
    
    output = torch.full((outy, outx, 3), default, dtype=int, device=device)
        # construct the output tensor
    output[ybuff : ybuff + newy, xbuff : xbuff + newx, :] = \
        torch.from_numpy(resized_image)
        # center the resized image in the output tensor

    return output

def preprocess_images(image_paths, input_dim, batch_size, CUDA=False):
    '''
    preprocesses the images for the neural network.

    ---args---
    image_paths : list[str]
        the paths to the images to process.
    input_dim : tuple[int, int]
        the input resolution (y, x) of images for the network.
    batch_size : int, optional (default=1)
        the size of batches that images should be in.

    ---returns---
    list[torch.tensor] : the tensor representation of the image batches, with
        image's aspect ratios maintained, and each batch having the shape 
        (batch_size, 3, *input_dim)
    list[cv2.Image] : the original images
    list[tuple[int,int]] : list of the image's original (x, y) dimensions
    '''
    # +++ set device variable
    if CUDA:
        device = 'cuda:0'
    else:
        device = 'cpu'

    # +++ setup some lists to store outputs
    out_batches = [] # output image batches
    out_originals = [] # output original images
    out_dims = [] # original image dimensions
    
    # +++ get ready for batching
    num_imgs = len(image_paths)
    num_batches = num_imgs // batch_size
    
    # +++ main loop to build batches
    for batch_i in range(num_batches):
        ### first some setup
        # first we setup a blank tensor to hold the batch
        batch = torch.zeros((batch_size, 3, *input_dim)).to(device)
        # then get the image paths for this batch
        batch_image_paths = image_paths[batch_i * batch_size : (batch_i+1) * batch_size]

        ### then loop to build the batch
        for img_i, path in enumerate(batch_image_paths):
            ### first some setup
            orig_img = cv2.imread(path) # read the image from the path
            dim = [orig_img.shape[1], orig_img.shape[0]] # (x,y) original image dimension
            img = image_resize(orig_img, input_dim, device=device) 
                # make the image array
            img = img.flip(2).permute(2,0,1).clone() 
                # a lot happens here, we flip the channels of the image, and
                # also rearrange the array from being (y, x, channels) to
                # (channels, y, x), like the network likes
            img = img.div(255.0).unsqueeze(0) # here, we divide by 255 
                # (normalize the pixel values) and unsqeeze, changing it from 
                # shape (c,y,x) to (1,c,y,x)
            
            ### now add image and dimensions to the output list
            out_originals.append(orig_img)
            out_dims.append(dim)

            ### and add the resized image to the batch
            batch[img_i] = img
        
        ### and add the batch to the output batches
        out_batches.append(batch)
    
    ### convert the image dimension list to a tensor
    out_dims = torch.FloatTensor(out_dims).to(device)

    return out_batches, out_originals, out_dims
        
def yolo_transform(prediction, input_dim, anchors, num_classes, CUDA=False):
    '''
    the yolo transformation function takes a result from the yolo network and
    transforms it to essentially get out the bounding boxes that the network
    has found, and which classes it believes to be in those bounding boxes.
    
    ---args---
    prediction : torch.Tensor
        the tensor output of the neural network, which should be a tensor of
        size (batch_size, num_predictions*(5+num_classes), grid_size, grid_size)
    input_dim : int
        the input dimensions of the images fed into the neural network
    anchors : list[tuple[int,int]]
        the list of bounding box sizes used to make predictions at this scale
    num_classes : int
        the number of classes the network is choosing from
    CUDA : bool, optional (default=False)
        weather or not the GPU should be used

    ---returns---
    torch.Tensor : the output of yolo at this step in the network, will have
        size (batch_size, grid_size**2 * num_predictions, num_classes + 5)
        where the 5 first elements in the last dimension are the center of the
        bounding box, the width and height, and the confidence score
    '''
    # +++ first do some setup stuff
    batch_size = prediction.size(0) # get the batch size
    stride = input_dim // prediction.size(2) # convolution stride size
    grid_size = input_dim // stride # size of the grid
    bounding_box_attrs = 5 + num_classes # number of attributes for the bounding box
    num_anchors = len(anchors) # number of anchors used to make predictions
    anchors = [(a[0]/stride, a[1]/stride) for a in anchors]
        # this essentially converts the anchors from pixels to number of strides
       
    # +++ now we mush around and reshape the prediction tensor
    # shape = (batch_size, num_anchors*bounding_box_attrs, grid_size, grid_size)
    prediction = prediction.reshape(batch_size, bounding_box_attrs * num_anchors, grid_size*grid_size)
    # shape = (batch_size, num_anchors*bounding_box_attrs, grid_size*grid_size)
    prediction = prediction.transpose(1,2).contiguous()
    # shape = (batch_size, grid_size*grid_size, num_anchors*bounding_box_attrs)
    prediction = prediction.reshape(batch_size, grid_size*grid_size*num_anchors, bounding_box_attrs)
    # shape = (batch_size, grid_size*grid_size*num_anchors, bounding_box_attrs)

    # +++ now we apply sigmoid to the xcenter, ycenter, and object confidence
    prediction[:,:,:2] = torch.sigmoid(prediction[:,:,:2])
        # this applies sigmoid to the xcenter and ycenter
    prediction[:,:,4] = torch.sigmoid(prediction[:,:,4])
        # and this applies it to the confidence score

    # +++ now we apply the offset to the center as perscribed by the network
        # step 1 is to make this tensor called x_y_offset, since the network
        # predicts the bounding box center's offset from the upper left corner
        # of each grid cell, the x_y_offset tensor will hold the upper left
        # corner coordinates for each grid cell so that we can add it to the
        # predictions we just sigmoided to get the absolute coordinates of bbs
        # NOTE that we will have to setup this tensor to match the size of the
        # predictions tensor, so it will have shape (batch_size, grid_size *
        # grid_size * num_anchors, 2) for the 2 coordinates of the grid cell
    grid_range = torch.arange(grid_size, dtype=torch.float64) 
        # make a range [0,...,grid_size]
    y_offset, x_offset = torch.meshgrid(grid_range, grid_range)
        # this makes two grids, one that holds x coordinates (x_offset) and one
        # that holds y coordinates (y_offset) for each cell in the grid
    x_offset = torch.DoubleTensor(x_offset).reshape(-1,1)
    y_offset = torch.DoubleTensor(y_offset).reshape(-1,1)
        # this transforms both of them into vectors, since that's how the 
        # prediction tensor holds them
    x_y_offset = torch.cat((x_offset, y_offset), 1)
    ###return x_y_offset
        # x_y_offset is the concatenation of x/y_offset along dimension 1, so it 
        # holds ordered coords [(x,y), (x,y), ...] for each cell in the grid
    x_y_offset = x_y_offset.repeat(1, num_anchors)
        # then we repeat these coordinates for the number of predictions that
        # each cell makes predicted by
    x_y_offset = x_y_offset.view(-1,2)
        # and put it into it's vectorized form, so now this holds num_predictions
        # copies of the x and y offset for each cell in the grid
    x_y_offset = x_y_offset.unsqueeze(0)
        # and this wraps the whole tensor in one more leading dimension so that
        # it can be directly added to the prediction tensor like so
    prediction[:,:,:2] += x_y_offset
        # so now the prediction accounts for the x/y offset of each cell and holds
        # the legit coordinates of each bounding box
    
    # +++ now we neet to apply the log operation to the anchors to update them
        # using the network's prediction for each grid cell
    anchors = torch.FloatTensor(anchors) # make it a float tensor
    anchors = anchors.repeat(grid_size*grid_size, 1).unsqueeze(0)
        # this makes copies of the anchor box sizes for each grid_cell
    prediction[:,:,2:4] = torch.exp(prediction[:,:,2:4])*anchors
        # this is the log operation that updates the anchor boxes for each grid cell
    
    # +++ now we softmax the class scores
    prediction[:,:,5:] = torch.sigmoid(prediction[:,:,5:])

    # +++ lastly, we need to rescale the xcenter, ycenter, bbx, and bby by the
        # stride length, transforming them back to absolute image coordinates
    prediction[:,:,:4] *= stride

    return prediction

def unique(tensor):
    '''
    accepts a 1-dimensional tensor and returns a 1-dimensional tensor where any
    repeated values have been removed.
    
    --- args ---
    tensor : torch.Tensor
    
    --- returns ---
    torch.Tensor
    '''
    nptensor = tensor.cpu().numpy() # convert to numpy
    out_tensor = np.unique(nptensor) # use np.unique
    out_tensor = torch.from_numpy(out_tensor) # convert back to pytorch

    out = tensor.new(out_tensor.shape) # make the output tensor (same dtype/device)
    out.copy_(out_tensor) # copy values from the np tensor
    return out

def get_box_area(*coords):
    ''' +++ INTERNAL FUNCTION +++
    gets the inner area of a box.
    '''
    x1, y1, x2, y2 = coords
    
    delta_x = torch.clamp(x2 - x1 + 1, 0)
    delta_y = torch.clamp(y2 - y1 + 1, 0)

    return delta_x * delta_y

def bbox_IoU(box, boxes):
    ''' +++ INTERNAL FUNCTION +++
    gets the intersect over union of one box with a set of other boxes two given
    boxes from NMS.
    '''
    # get the coordinates for all the boxes
    bx1, by1, bx2, by2 = box[:,:4].reshape(4) # get coordinates for box
    bsx1, bsy1, bsx2, bsy2 = boxes[:,:4].transpose(0,1) # get coords for all boxes

    # get the coordinates for the intersect box
    x1 = torch.max(bx1, bsx1)
    y1 = torch.max(by1, bsy1)
    x2 = torch.min(bx2, bsx2)
    y2 = torch.min(by2, bsy2)
    
    # get the intersection area and box areas
    box_area = get_box_area(bx1, by1, bx2, by2)
    boxes_area = get_box_area(bsx1, bsy1, bsx2, bsy2)
    intersect_area = get_box_area(x1, y1, x2, y2)

    # get the iou!
    iou = intersect_area / (box_area + boxes_area - intersect_area)

    return iou

def c1c2_iou_mat(boxes1, boxes2):
    x11, y11, x12, y12 = boxes1.t()
    x21, y21, x22, y22 = boxes2.t()

    bx1_area, bx2_area = torch.meshgrid((x12-x11)*(y12-y11),
                                        (x22-x21)*(y22-y21))

    ix1 = torch.max(*torch.meshgrid(x11, x21))
    iy1 = torch.max(*torch.meshgrid(y11, y21))
    ix2 = torch.min(*torch.meshgrid(x12, x22))
    iy2 = torch.min(*torch.meshgrid(y12, y22))

    iw = ix2 - ix1
    ih = iy2 - iy1
    int_area = (iw * ih) * (iw > 0) * (ih > 0)

    union_area = bx1_area + bx2_area - int_area
    
    iou_mat = int_area / union_area

    return iou_mat

def nms_prediction(prediction, nms_thresh=0.4):
    ''' +++ INTERNAL FUNCTION +++
    does nms on the prediction coming from get_results.
    '''
    num_detections = prediction.size(0) # number of predictions
    i = 0 # index counter
    # go through each detection

    while i < prediction.size(0):
        # get the ious of all the boxes after this one
        ious = bbox_IoU(prediction[i].unsqueeze(0), prediction[i+1:])
        # get the indexs for detections with IoU < nms_thresh, i.e. the
        # detections that we should continue to include in the batch
        iou_idxs = torch.where(ious < nms_thresh)[0]
        iou_idxs = iou_idxs + i + 1 # add i+1 to get index w/r/t prediction

        # replace the prediction with the ones that are above the threshold
        prediction = torch.cat((
            prediction[:i+1], # all the predictions up to and including this one
            prediction[iou_idxs] # all the predictions after, above the threshold
        ), 0)
        i += 1

    return prediction
    
def get_results(prediction, conf_thresh, num_classes, nms=True, nms_thresh=0.4):
    '''
    this function takes in the output from the network and figures out what
    bounding boxes it is actually trying to predict within the image.

    --- args ---
    prediction : torch.Tensor
        the output from the network, shape should be (batch_size, 3, image_y, 
        image_x).
    conf_thresh : float
        confidence threshold, only bounding boxes with a confidence value above
        this number will be considered.
    num_classes : int
        the number of classes that the network is choosing from.
    nms : bool, optional (default=True)
        should non-maximum supression be run on the returned bounding boxes.
    nms_thresh : float, optional (default=0.4)
        the non-max supression confidence threshold.
    '''
    # +++ first, we make the confidence mask, which collapses all bounding box
        # predictions that have a confidence value below our threshold to zero
    conf_mask = (prediction[:,:,4] > conf_thresh).float().unsqueeze(2)
    prediction = prediction*conf_mask
    '''
    # +++ now, we get the precice indicies of all the bounding box predictions
        # that remain
    try:
        # get the non zero indices
        nz_idxs = torch.nonzero(prediction[:,:,4], as_tuple=False)
        # transpose them, so that they are two lists of [y,y,...],[x,x,...]
        nz_idxs = nz_idxs.transpose(0,1).contiguous()
    except:
        # if that throws an error, it's because there were no predictions that
        # were made, so return None
        return None
    '''
    # +++ now, prediction is storing the box's centerx, centery, width, height
        # but we'd like it to store x_min, y_min, x_max, y_max, so we do this
    
    x_center = prediction[:,:,0].clone()    # x center coordinates
    y_center = prediction[:,:,1].clone()    # y center coords
    width = prediction[:,:,2].clone()       # box widths
    height = prediction[:,:,3].clone()      # box heights
    
    prediction[:,:,0] = x_center - width/2 # x min
    prediction[:,:,1] = y_center - height/2 # y min
    prediction[:,:,2] = x_center + width/2 # x max
    prediction[:,:,3] = y_center + height/2 # y max

    # +++ now we iterate over each image in the batch
    batch_size = prediction.size(0) # get the size of the batch
    output = None # output variable (will hold a tensor)

    for img_i in range(batch_size):
        image_pred = prediction[img_i] # select the image from the batch
        
        # +++ now we will replace the last num_classes items in the image_pred tensor
            # with two numbers, the class that we are most confident about, and the
            # index of that class, making the image_pred tensor have shape
            # (num_preds, 7) by the end of this chunk of code
        cls_pred_conf, cls_preds = torch.max(image_pred[:,5:], 1)
        cls_pred_conf = cls_pred_conf.float().unsqueeze(1)
        cls_preds = cls_preds.float().unsqueeze(1)
        image_pred = torch.cat((image_pred[:,:5], cls_pred_conf, cls_preds), 1)

        # +++ now we will get rid of all the entries that have a confidence value
            # below our threshold, which have already been colapsed to zero, so
            # we can get their indices with
        non_zero_idxs = torch.nonzero(image_pred[:,4], as_tuple=False)
        # and then keep just the detections made at indices above
        image_pred = image_pred[non_zero_idxs.squeeze(), :].view(-1, 7)

        # +++ since we are doing NMS class-wise, we need to figure out what classes
            # are present in the image
        img_classes = unique(image_pred[:,-1]) # only want unique classes
        
        # +++ now loop through classes for NMS
        for cls_n in img_classes:
            # get all the detections made for that class
            cls_idxs = torch.where(image_pred[:,-1] == cls_n)[0] # get the indices
            image_pred_class = image_pred[cls_idxs] # only keep those rows

            # now we want to sort the detections by object confidence (index 4)
            conf_sorted_idxs = torch.sort(image_pred_class[:,4], descending=True)[1]
            image_pred_class = image_pred_class[conf_sorted_idxs]

            # and if we need to do nms, then we do nms
            if nms:
                image_pred_class = nms_prediction(image_pred_class, nms_thresh)
            
            # lastly, we need to keep track of which image in each batch each
                # detection belongs to, so we will add a column to the front that 
                # stores just that index
            img_i_col = image_pred_class.new(image_pred_class.size(0), 1).fill_(img_i)
                # make a solid column of the image's index within the batch
            image_pred_class = torch.cat((img_i_col, image_pred_class), 1)
                # concatenate it to the front of the predictions for this class
            # now, we write the output
            if output == None: # if output hasn't been written yet
                output = image_pred_class.clone() # just replace it with predictions
            else: # otherwise
                output = torch.cat((output, image_pred_class), 0) # add it to the end
    
    return output

def wh_iou_mat(box_wh1, box_wh2):
    ''' +++ INTERNAL FUNCTION +++ 
    this function creates a matrix of IOU values between two sets of box widths
    and heights, assuming that they share a center.

    --- args ---
    box_wh1 : torch.Tensor of shape (num_boxes1, 2)
        the first set of box widths and heights.
    targets : torch.Tensor of shape (num_boxes2, 2)
        the second set of box widths and heights.
    
    --- returns ---
    torch.Tensor of shape (num_boxes1, num_boxes2) : the IOUs for each 
        combination of anchor box and target box, where the index [i,j] 
        corresponds to the ith anchor box and jth target box given.
    '''
    # make copies of the tensors
    box_wh1 = box_wh1.clone().float()
    box_wh2 = box_wh2.clone().float()

    # unpack the inputs
    bx1_w, bx1_h = box_wh1[:,0], box_wh1[:,1]
    bx2_w, bx2_h = box_wh2[:,0], box_wh2[:,1]

    # get the areas for each anchor box/target box
    bx1_areas, bx2_areas = torch.meshgrid(box_wh1.prod(1), box_wh2.prod(1))
        # note: this also puts the areas in matrix format
    
    # calculate the intersection areas
    int_widths = torch.min(*torch.meshgrid(bx1_w, bx2_w))
    int_heights = torch.min(*torch.meshgrid(bx1_h, bx2_h))
    intersect_areas = int_widths * int_heights
    
    # calculate union areas
    union_areas = bx1_areas + bx2_areas - intersect_areas

    # return intersect over union
    return intersect_areas/union_areas

# FIXME: REMOVE THIS FUNCTION
def bbox_wh_iou(anchors, targets):
    ''' +++ INTERNAL FUNCTION +++ 
    this function calculates the IOU of a set of anchor boxes and target boxes,
    assuming that the boxes share a center.

    --- args ---
    anchors : torch.Tensor of shape (num_anchors, 2)
        the anchor box's widths and heights.
    targets : torch.Tensor of shape (num_targets, 2)
        the target box's widths and heights.
    
    --- returns ---
    torch.Tensor of shape (num_anchors, num_targets) : the IOUs for each 
        combination of anchor box and target box, where the index [i,j] 
        corresponds to the ith anchor box and jth target box given.
    '''
    print('bbox_wh_iou FUNCTION IS DEPRICIATED')
    # make float copies of the tensors
    anchors = anchors.clone().float()
    targets = targets.clone().float()

    # unpack the inputs
    nt = targets.size(0) # number of targets
    na = anchors.size(0) # number of anchors
    aw, ah = anchors[:,0], anchors[:,1] # anchor widths and heights
    tw, th = targets[:,0], targets[:,1] # target widths and heights

    # calculate areas of anchors and targets
    a_areas = anchors.prod(1) # just multiply width and height for each box
    t_areas = targets.prod(1)

    # calculate the intersection areas
    min_widths = torch.min(*torch.meshgrid(aw, tw)) # minimum widths
    min_heights = torch.min(*torch.meshgrid(ah, th)) # minimum heights
    intersect_areas = min_widths * min_heights

    # calculate union areas
    a_areas_, t_areas_ = torch.meshgrid(a_areas, t_areas) # this basically duplicates the
        # anchor box and target box areas into a shape compatible with the intersect areas
    union_areas = a_areas_ + t_areas_ - intersect_areas

    # return intersect over union
    return intersect_areas/union_areas

def bbox_xywh_ious(boxes1, boxes2):
    ''' +++ INTERNAL FUNCTION +++ 
    takes in two sets of boxes and computes their IOUs with one-another.
    
    --- args ---
    boxes1 : torch.Tensor of shape (num_boxes, 4)
        the first set of boxes, with the first dimension corresponding to
        (center_x, center_y, width, height).
    boxes2 : torch.Tensor of shape (num_boxes, 4)
        the second set of boxes, with the same formatting as boxes1.
    '''
    # +++ get number of boxes
    nbs = boxes1.size(0)
    assert nbs == boxes2.size(0)

    # +++ first step is to get the corner coordinates for each box
    bxcs1 = bbox_xywh_getcs(boxes1)
    bxcs2 = bbox_xywh_getcs(boxes2)

    # +++ find the corner coordinates of the intersection boxes
    int_bx_cs = torch.zeros((nbs, 4)).to(boxes1.device)
    
    int_bx_cs[:,:2] = torch.max(bxcs1, bxcs2)[:,:2] # use max of the first
        # corner coordinates to get it for intersection
    int_bx_cs[:,2:] = torch.min(bxcs1, bxcs2)[:,2:] # use min of the second
        # corner coordinates to get it for intersection
    
    # +++ calculate intersection area
    int_w = int_bx_cs[:,2] - int_bx_cs[:,0] # intersection widths
    int_h = int_bx_cs[:,3] - int_bx_cs[:,1] # intersection heights
    int_w = torch.clamp(int_w, 0) # no boxes with negative widths
    int_h = torch.clamp(int_h, 0) # or negative heights
    int_A = int_w * int_h # intersection areas

    # +++ calculate union areas
    bxs1_A = boxes1[:,2:].prod(1) # boxes1 areas (width*height)
    bxs2_A = boxes2[:,2:].prod(1) # boxes2 areas
    union_A = bxs1_A + bxs2_A - int_A # union areas

    # return intersection over union
    return int_A / union_A
    
def bbox_xywh_getcs(boxes):
    ''' +++ INTERNAL FUNCTION +++ 
    gets corners of some bounding boxes from their x, y, widths and heights.
    
    --- args ---
    boxes : torch.Tensor of shape (num_boxes, 4)
        the set of boxes, with the first dimension corresponding to
        (center_x, center_y, width, height).
    
    --- returns ---
    torch.Tensor of shape (num_boxes, 4) where the first dimension corresponds
        to (x1, y1, x2, y2) of the corners
    '''
    centers = boxes[:,:2].float() # get centers
    wh = boxes[:,2:].float() # get widths and heights
    
    out = centers.repeat(1,2) # repeat centers twice
    out[:,:2] -= .5 * wh  # subtract half of widths and heights from first
        # set of centers to get (x1, y1)
    out[:,2:] += .5 * wh # add half of widths and heights to second set of
        # centers to get (x2, y2)
    
    return out

###