'''
this file holds network block classes that can be constructed from dictionaries
assembled from the config files.
'''
import torch
import torch.nn as nn
import torch.nn.functional as F

# +++ global functions
def get_activ(activation):
    ''' gets the activation function from the string provided.
    
    --- args ---
    activation : str
        the string name of the activation function.
    
    --- returns ---
    torch.nn.Module : the activation function requested.
    OR
    None : if activation = 'linear'
    '''
    if activation == 'relu':
        act = nn.ReLU() # relu activation
    elif activation == 'leaky':
        act = nn.LeakyReLU(0.1)
    elif activation == 'sigmoid':
        act = nn.Sigmoid()
    elif activation == 'linear':
        act = nn.Identity()
    return act

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

# +++ EXAMPLE BLOCK_DICT
block_dict = {
    'layer_num': 0,
    'type': 'detection',
    'name': 'foo',
    'anchors': [[12, 26], [14, 29], [15, 34]], # DONT USE THESE FUCK I NEED TO FIGURE THIS OUT FOR THE DATA
    'ignore_thresh': 0.7,
    'lambda_bbox': 1.0,
    'lambda_noobj': 1.00,
    'lambda_obj': 1.0,
    'lambda_cls': 1.0,
    'input_dim': 256,
    'classes': 21,
    'lbl_smoothing': 0.05}

# +++ network block classes
class DetBlock(nn.Module):
    ''' detection block
    a detection block is at the core of both YOLO and SqueezeDet architectures.
    it is essentially just a bunch of transformations that get applied to the
    previous output to interpret it as detections of bounding boxes for the
    network. this block is also set up to compute the loss of the detections
    alongside the transformations which helps a LOT with network design.

    --- args ---
    block_dict : dict
        the dictionary that describes this yolo block. should contain keys like
        'input_dim', 'anchors', 'classes', and 'ignore_thresh'.
    CUDA : bool, optional (default=False)
        should CUDA be used? this can be modified later by changing the device 
        variable for an instance of this class.
    '''
    def __init__(self, block_dict, CUDA=False):
        # +++ super init!
        super(DetBlock, self).__init__()
        
        # +++ set the device variable
        self.device = 'cuda:0' if CUDA else 'cpu'
        
        # +++ save the block dict and attributes
        self.block_dict = block_dict
        self.n = block_dict['layer_num']
        self.type = block_dict['type']

        # +++ save more inputs from block_dict
        self.img_dim = block_dict['input_dim']
        self.anchors = block_dict['anchors']
        self.num_anchors = len(self.anchors)
        self.num_classes = block_dict['classes']
        self.ignore_thresh = block_dict['ignore_thresh']
        self.lbl_smth_e = block_dict['lbl_smoothing']
        self.CUDA = CUDA
        
        # +++ initialize some variables
        # loss functions
        self.mse_loss = nn.MSELoss()
        self.bce_loss = nn.BCELoss()
        # loss lambdas
        self.lambda_obj = block_dict['lambda_obj']
        self.lambda_noobj = block_dict['lambda_noobj']
        self.lambda_bbox = block_dict['lambda_bbox']
        self.lambda_cls = block_dict['lambda_cls']
        # other things that will be set/updated later
        self.metrics = {}
        self.grid_size = 0
        self.stride = 0
    
    # +++ compute grid offsets method
    def compute_grid_offsets(self, grid_size):
        ''' +++ INTERNAL METHOD +++
        computes the grid offsets for each cell in a given grid_size. these get
        stored as the grid_x and grid_y variables.
        '''
        self.grid_size = grid_size # update self.grid_size
        self.stride = self.img_dim / self.grid_size # calculate stride
        g = grid_size # save grid_size as g

        # +++ calculate offsets for each cell in the grid
        grid_range = torch.arange(g).to(self.device) # range of the grid
        self.grid_y, self.grid_x = torch.meshgrid(grid_range, grid_range) # y/x offsets
        
        # unsqueeze and make float tensors
        self.grid_x = self.grid_x.view(1,1,g,g).float()
        self.grid_y = self.grid_y.view(1,1,g,g).float()

        # +++ calculate anchors normalized to the stride
        self.scaled_anchors = torch.FloatTensor(self.anchors).to(self.device)
        self.scaled_anchors /= self.stride

        # +++ get the anchor w and anchor heights
        self.anchor_w = self.scaled_anchors[:,0:1].view(1, self.num_anchors, 1, 1)
        self.anchor_h = self.scaled_anchors[:,1:2].view(1, self.num_anchors, 1, 1)
    
    # +++ forward (implicit __call__) method
    def forward(self, x, targets=None):
        ''' feed forward method
        takes in the output of the network so far and performs transformations 
        to interpret it as a set of detections the network is making.

        additionally, if a targets are passed, the loss of the network will be
        calculated alongside the output as it feeds through.

        --- args ---
        x : torch.Tensor
            the output of the network thus far, which should have shape
            (batch_size, (num_classes+5)*num_anchors, grid_size, grid_size)
        targets : torch.Tensor
            the target predictions for this batch of data, which should have
            shape (num_objects, 6) where the first dimension contains 
            [img_number, class_number, x, y, w, h] for each object in the batch,
            where x, y, w, and h are all normalized by the image dimension.
        
        --- returns ---
        torch.Tensor : the predictions made by this detection layer, with shape
            (batch_size, grid_size**2 * num_anchors, num_classes + 5).
        float or torch.Tensor : the loss of the network over this batch (if 
            targets) were given. if no targets were given it will be 0.
        '''
        # +++ get info from the input tensor size
        self.batch_size = x.size(0)
        grid_size = x.size(2)

        # +++ compute the grid offsets if the grid_size changed
        if grid_size != self.grid_size:
            self.compute_grid_offsets(grid_size)
        
        # +++ reformat x to get predictions
        # first we break up dim 1 into num_anchors and 5+num_classes
        prediction = x.view(self.batch_size, self.num_anchors, self.num_classes+5, grid_size, grid_size)
        # then we want to move the 5+num_classes dimension to the back, so
        prediction = prediction.permute(0, 1, 3, 4, 2).contiguous()

        # +++ do the first set of transformations on predictions to get it
            # ready to calculate the final bounding boxes and the loss
        # sigmoid x,y, and box conf and class confs
        prediction[...,:2] = torch.sigmoid(prediction[...,:2])
        prediction[...,4:] = torch.sigmoid(prediction[...,4:])

        # +++ make the output tensor and tidy it up
        output = prediction.clone()
        # add offsets to x and y
        output[..., 0] += self.grid_x
        output[..., 1] += self.grid_y
        # exponentiate and scale w/h
        output[..., 2:4] = torch.exp(output[..., 2:4])
        output[..., 2] *= self.anchor_w
        output[..., 3] *= self.anchor_h
        # save pred boxes and classes for calculating loss
        box_preds = output[...,:4].clone()
        cls_preds = output[...,5:].clone()
        # scale x, y, w, h to be in units of pixels
        output[..., :4] *= self.stride
        # make it a just a list of detections
        output = output.view(self.batch_size, -1, self.num_classes+5)

        # +++ if no targets were given then return here
        if targets == None:
            return output, 0    
        
        ##############################
        ###    LOSS CALCULATION    ###
        ##############################

        # +++ use the build targets method to build the targets for this input
        obj_mask, noobj_mask, txywh, tcls, tconf, correct_cls, iou_scores = \
            self.build_targets(targets, cls_preds, box_preds)

        # +++ calculate some losses, using the masks to ignore certain outputs
            # that we don't care about/don't want to affect our loss
        # initialize all losses to zero
        loss_conf_obj, loss_conf_noobj, loss_bbox, loss_cls = torch.zeros(4)

        ### noobj confidence loss
        loss_conf_noobj = self.bce_loss(
            prediction[...,4][noobj_mask], tconf[noobj_mask]) * self.lambda_noobj
        loss_conf = loss_conf_noobj

        ### this conditional protects against nan loss values
        if obj_mask.any():
            ### obj/noobj confidence loss
            loss_conf_obj = self.bce_loss(
                prediction[...,4][obj_mask], tconf[obj_mask]) * self.lambda_obj
            loss_conf += loss_conf_obj

            ### bounding box loss
            loss_bbox = self.mse_loss(prediction[...,0:4][obj_mask], txywh[obj_mask]) \
                * self.lambda_bbox
                    
            ### class loss
            loss_cls = self.bce_loss(
                prediction[...,5:][obj_mask], tcls[obj_mask]) * self.lambda_cls

        ### sum above to get total loss
        total_loss = loss_bbox + loss_conf + loss_cls

        ##############################
        ###  METRICS CALCULATIONS  ###
        ##############################

        # +++ a few helper masks for the metrics
        conf50 = (prediction[...,4] > 0.5).float()
            # where box confidence was > 50%
        iou50 = (iou_scores > 0.5).float()
            # correctly predicted bounding box with iou > 0.5
        iou75 = (iou_scores > 0.75).float()
            # correctly predicted bounding box with iou > 0.75
        detected_mask = conf50 * correct_cls * obj_mask
            # mask for objects being "detected"

        # +++ calculate some metrics !!!
        cls_acc = correct_cls[obj_mask].mean()
            # average class accuracy where there is an object
        conf_obj = prediction[...,4][obj_mask].mean()
            # average confidence where there was an object present
        conf_noobj = prediction[...,4][noobj_mask].mean()
            # average confidence where there was no object present
        precision = torch.sum(iou50 * detected_mask) / (conf50.sum() + 1e-16)
            # precision = # of detected objs w/ iou > 0.5 / # of conf vals > 0.5
        recall50 = torch.sum(iou50 * detected_mask) / (obj_mask.sum() + 1e-16)
            # % of objs that were detected w/ iou > 0.5
        recall75 = torch.sum(iou75 * detected_mask) / (obj_mask.sum() + 1e-16)
            # % of objs that were detected w/ iou > 0.75
        
        # +++ lastly, calculate class accuracy BY class
        cls_acc_by_cls = torch.zeros(self.num_classes)
        for cls_n in range(self.num_classes):
            n_true = (targets[:,1] == cls_n).sum()
                # true number of that class in the images
            n_pred = (cls_preds.argmax(-1)[correct_cls.bool()] == cls_n).sum()
                # number of that class in image that were correctly predicted
            cls_acc_by_cls[cls_n] = (n_pred/(n_true + 1e-16)).item()

        # +++ save the metrics
        self.metrics = {
            'loss': total_loss.item(),
            'bbox-loss': loss_bbox.item(),
            'noobj-conf-loss': loss_conf_noobj.item(),
            'obj-conf-loss': loss_conf_obj.item(),
            'conf-loss': loss_conf.item(),
            'cls-loss': loss_cls.item(),
            'cls-accuracy': cls_acc.item(),
            'cls-acc-by-cls': cls_acc_by_cls,
            'recall50': recall50.item(),
            'recall75': recall75.item(),
            'precision': precision.item(),
            'conf-obj': conf_obj.item(),
            'conf-noobj': conf_noobj.item(),
            'grid-size': self.grid_size}

        # return the output and the loss
        return output, total_loss
    
    # +++ build targets method
    def build_targets(self, targets, cls_preds, box_preds):
        ''' +++ INTERNAL METHOD +++ 
        this function takes in some targets in the form that datasets usually
        give them, i.e.
        [[image_num, class_num, center_x, center_y, width, height],...]
        and creates the masks that can be used to...
        '''
        # +++ set the tensor variables
        BoolTensor = torch.cuda.BoolTensor if self.CUDA else torch.BoolTensor
        FloatTensor = torch.cuda.FloatTensor if self.CUDA else torch.FloatTensor

        # +++ setup the output tensors
        # many masks have the same size, so this variable is helpful
        mask_size = (self.batch_size, self.num_anchors, self.grid_size, self.grid_size)
        # now we make all the tensors
        obj_mask = BoolTensor(*mask_size).fill_(0)
        noobj_mask = BoolTensor(*mask_size).fill_(1)
        tx = FloatTensor(*mask_size).fill_(0)
        ty = FloatTensor(*mask_size).fill_(0)
        tw = FloatTensor(*mask_size).fill_(0)
        th = FloatTensor(*mask_size).fill_(0)
        tcls = FloatTensor(*mask_size, self.num_classes).fill_(
            self.lbl_smth_e / (self.num_classes-1))
        correct_cls = FloatTensor(*mask_size).fill_(0)
        iou_scores = FloatTensor(*mask_size).fill_(0)

        # +++ make sure there are targets in this batch
        if targets.numel() == 0:
            # no objects, just return the masks as they are
            tconf = obj_mask.float()
            txywh = torch.stack((tx, ty, tw, th), dim=-1)
            return obj_mask, noobj_mask, txywh, tcls, tconf, correct_cls, iou_scores

        # +++ convert the target coordinates to the grid's scale
        target_boxes = targets[:,2:6] * self.grid_size
        gxy = target_boxes[:,:2] # target box center x and ys
        gwh = target_boxes[:,2:] # target box widths and heights

        # +++ find the anchor box with the best IOU for each target
        ious = wh_iou_mat(self.scaled_anchors, gwh)
        best_anchs = ious.max(0)[1]
        
        # +++ unpack the target values
        img_i, target_lbls = targets[:,:2].long().t() # img index & class labels
        gx, gy = gxy.t()
        gw, gh = gwh.t()
        gi, gj = gxy.long().t() # these are the grid indicies responsible for 
            # detecting each target given
        
        # +++ set the obj/noobj mask values where there is an object
        obj_mask[img_i, best_anchs, gj, gi] = 1
        noobj_mask[img_i, best_anchs, gj, gi] = 0

        # +++ set the noobj mask to 0 where the IOU exceeds the threshold
        for i, anchor_ious in enumerate(ious.t()):
            noobj_mask[img_i[i], anchor_ious > self.ignore_thresh, gj[i], gi[i]] = 0

        # +++ set the targets for the x/y predictions
        tx[img_i, best_anchs, gj, gi] = gx - gx.floor()
        ty[img_i, best_anchs, gj, gi] = gy - gy.floor()

        # +++ set targets for w/h predictions
        tw[img_i, best_anchs, gj, gi] = \
            torch.log(gw / self.scaled_anchors[best_anchs][:,0] + 1e-16)
        th[img_i, best_anchs, gj, gi] = \
            torch.log(gh / self.scaled_anchors[best_anchs][:,1] + 1e-16)
        
        # +++ one-hot encoding of class labels
        tcls[img_i, best_anchs, gj, gi, target_lbls] = 1 - self.lbl_smth_e

        # +++ target conf should just be the same as object mask
        tconf = obj_mask.float()

        # +++ a few more things for metrics
        correct_cls[img_i, best_anchs, gj, gi] = \
            (cls_preds[img_i, best_anchs, gj, gi].argmax(-1) == target_lbls).float()
            # holds a 0 if the cell did not need to predict a class or got it wrong and
            # a 1 iff the responsible cell got the class correct
        
        iou_scores[img_i, best_anchs, gj, gi] = bbox_xywh_ious(
            box_preds[img_i, best_anchs, gj, gi], target_boxes)
            # set the iou scores for the cells/anchor boxes responsible for predicting
            # the targets to be the IOU of the predicted box with the target boxes
        
        txywh = torch.stack((tx, ty, tw, th), dim=-1)
        return obj_mask, noobj_mask, txywh, tcls, tconf, correct_cls, iou_scores

    # +++ end of YoloBlock class
