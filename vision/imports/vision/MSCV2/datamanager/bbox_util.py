''' bbox_util.py
this file provides a handful of helpful functions for dealing with and 
manipulating bounding boxes.
'''
# +++ IMPORTS
import torch

# +++ FUNCTIONS
def bb_xywh_to_cs(boxes, inplace=True):
    ''' bounding box xywh to corners
    this function takes in bounding box information in the x,y,w,h format and
    spits it back in the xmin, ymin, xmax, ymax (corners) format.

    --- args ---
    boxes : torch.Tensor like (_, 4)
        the input x,y,w,h data for the bounding boxes.
    inplace : bool, optional (defualt=True)
        if true, the operation will manipulate the input tensor itself instead
        of creating another.

    --- returns ---
    torch.Tensor like (_, 4) : the xmin, ymin, xmax, ymax data for the same
        bounding boxes.
    '''
    xc, yc, w, h = boxes.clone().t()
    if not inplace:
        boxes = torch.zeros_like(boxes)
    boxes[:,0] = xc - w/2
    boxes[:,1] = yc - h/2
    boxes[:,2] = xc + w/2
    boxes[:,3] = yc + h/2
    return boxes

def bb_cs_to_xywh(boxes, inplace=True):
    ''' bounding box corners to xywh
    this function takes in bounding box information in the xmin, ymin, xmax,
    ymax (corners) and returns it in the x,y,w,h format.

    --- args ---
    boxes : torch.Tensor like (_, 4)
        the input xmin, ymin, xmax, ymax data for the bounding boxes.
    inplace : bool, optional (defualt=True)
        if true, the operation will manipulate the input tensor itself instead
        of creating another.

    --- returns ---
    torch.Tensor like (_, 4) : the x,y,w,h data for the same bounding boxes.
    '''
    xmin, ymin, xmax, ymax = boxes.clone().t()
    if not inplace:
        boxes = torch.zeros_like(boxes)
    boxes[:,0] = (xmin + xmax) / 2
    boxes[:,1] = (ymin + ymax) / 2
    boxes[:,2] = (xmax - xmin)
    boxes[:,3] = (ymax - ymin)
    return boxes

def bb_cs_iou(boxes1, boxes2):
    ''' bounding box corners IoU
    this calculates the pairwise IoU between two sets of bounding boxes.

    --- args ---
    boxes1 : torch.Tensor like (N, 4)
        the first set of boxes in the xmin, ymin, xmax, ymax format.
    boxes2 : torch.Tensor like (N, 4)
        the second set of boxes in the xmin, ymin, xmax, ymax format.

    --- returns ---
    torch.Tensor like (N,) : the intersect over union between adjacent bounding
        boxes in the input.
    '''
    # unpack corners
    b1c1s = boxes1[:, :2]
    b1c2s = boxes1[:, 2:]
    b2c1s = boxes2[:, :2]
    b2c2s = boxes2[:, 2:]
    # get the intersection corners
    intc1s = torch.max(b1c1s, b2c1s)
    intc2s = torch.min(b1c2s, b2c2s)
    # get the box areas
    b1As = (b1c2s - b1c1s).prod(1)
    b2As = (b2c2s - b2c1s).prod(1)
    # get intersection areas
    intwh = (intc2s - intc1s)
    intwh *= (intwh > 0).all(1).unsqueeze(1) # zero out dims < 0
    intAs = intwh.prod(1)
    # get union areas
    uniAs = b1As + b2As - intAs
    return intAs/uniAs
