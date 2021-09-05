'''
this file holds network block classes that can be constructed from dictionaries
assembled from the config files.
'''
import torch
import torch.nn as nn
import torch.nn.functional as F
from util import make_tup, wh_iou_mat, bbox_xywh_ious

# +++ custom activation functions


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

class ConvolutionalBlock(nn.Sequential):
    ''' convolutional network block
    a convolutional block applies convolution (obviously), along with an 
    activation function and batch normalization, as those are provided.

    --- args ---
    block_dict : dict
        the dictionary that describes this convolutional block.
    '''
    def __init__(self, block_dict):
        super(ConvolutionalBlock, self).__init__() # run super init

        # save the block dict and attributes
        self.block_dict = block_dict # save block dictionary
        self.n = block_dict['layer_num'] # save block number
        self.type = block_dict['type'] # save block type

        # make the convolutional part
        conv = nn.Conv2d(
            block_dict['in_channels'], 
            block_dict['filters'], 
            kernel_size = block_dict['kernel_size'],
            stride = block_dict['stride'],
            padding = block_dict['padding'],
            bias = block_dict['bias'])

        # add conv part to the module
        self.add_module('convolution', conv)

        # maybe make and add batch normalization
        if block_dict['batch_normalize']:
            batch_norm = nn.BatchNorm2d(block_dict['filters']) # make the part
            self.add_module('batch norm', batch_norm) # add it
        
        # add the activation function
        activation = get_activ(block_dict['activation'])
        self.add_module(block_dict['activation'], activation)

class FireBlock(nn.Module):
    ''' fire block
    this is a fire block as described in the paper for sqeeze net.
    
    --- args ---
    block_dict : dict
        the config block dictionary.
    '''
    def __init__(self, block_dict):
        super(FireBlock, self).__init__() # run super init
        # unpack input
        self.n = block_dict['layer_num'] # save layer number
        in_chan = block_dict['in_channels']
        fs = block_dict['fsqueeze']
        fe = block_dict['fexpand']
        
        # make the layers of the module
        self.squeeze = nn.Conv2d(in_chan, fs, kernel_size=1)
        self.expand1 = nn.Conv2d(fs, fe, 1)
        self.expand3 = nn.Conv2d(fs, fe, 3, padding=1)

        # batch normalization
        if block_dict['batch_normalize']:
            self.batch_norm = nn.BatchNorm2d(fe * 2)
        else:
            self.batch_norm = None
        
        # get the activation function to use
        self.activ = get_activ(block_dict['activation'])
    
    def forward(self, x):
        s = self.activ(self.squeeze(x)) # squeeze x
        e1 = self.activ(self.expand1(s)) # expand 1
        e3 = self.activ(self.expand3(s)) # expand 3
        output = torch.cat((e1, e3), dim=1) # concatenate expand layers
        if self.batch_norm != None:
            output = self.batch_norm(output)
        return output

class MaxPoolBlock(nn.Module):
    ''' max pooling block
    a block of the network that performs max pooling.

    --- args ---
    block_dict : dict
        the dictionary that describes this max pooling block. should contain 
        keys like 'kernel_size', 'stride', and 'padding'.
    '''
    def __init__(self, block_dict):
        super(MaxPoolBlock, self).__init__() # super init

        # save the block dict and attributes
        self.block_dict = block_dict # save block dictionary
        self.n = block_dict['layer_num'] # save block number
        self.type = block_dict['type'] # save block type

        # save the given variables
        self.kernel_size = block_dict['kernel_size']
        self.stride = block_dict['stride']
        self.padding = make_tup(block_dict['padding'], tup_len=4, expand_method=1)

    def forward(self, x):
        return F.max_pool2d(
            F.pad(
                x, 
                pad = self.padding,
                mode = 'replicate'),
            kernel_size = self.kernel_size,
            stride = self.stride)
    
class UpsampleBlock(nn.Upsample):
    ''' upsample block
    a network block that performs upsampling.
    
    --- args ---
    block_dict : dict
        the dictionary that describes this block. should have key 
        'scale_factor'.
    '''
    def __init__(self, block_dict):
        # initialize urself
        super(UpsampleBlock, self).__init__(
            scale_factor = block_dict['scale_factor'],
            mode = 'bilinear',
            align_corners = False)
        
        # save the block dict and attributes
        self.block_dict = block_dict # save block dictionary
        self.n = block_dict['layer_num'] # save block number
        self.type = block_dict['type'] # save block type
    
class RouteBlock(nn.Module):
    ''' route block
    a block of the network that performs routing.

    --- args ---
    block_dict : dict
        the dictionary that describes this routing block. should have key 
        'layers'.
    '''
    def __init__(self, block_dict):
        super(RouteBlock, self).__init__() # super init

        # save the block dict and attributes
        self.block_dict = block_dict # save block dictionary
        self.n = block_dict['layer_num'] # save block number
        self.type = block_dict['type'] # save block type

        # save routing layer numbers
        self.layers = block_dict['layers']
    
    def forward(self, layer_activations):
        # keep only the ones we want to route together
        to_route = [layer_activations[i] for i in self.layers]
        return torch.cat(to_route, dim=1)

class ShortcutBlock(nn.Module):
    ''' shortcut block
    a block that describes a shortcut in the network.
    
    --- args ---
    block_dict : dict
        the dictionary that describes this block of the network. should have key
        'from'.
    '''
    def __init__(self, block_dict):
        super(ShortcutBlock, self).__init__() # super init
        
        # save the block dict and attributes
        self.block_dict = block_dict # save block dictionary
        self.n = block_dict['layer_num'] # save block number
        self.type = block_dict['type'] # save block type

        # save layer to shortcut from
        self.layer = block_dict['from']
    
    def forward(self, *layer_activations):
        return sum(layer_activations)

class DropBlock2D(nn.Module):
    ''' dropblock layer 
    a dropblock layer performs dropblock on the incoming data, if and only if
    the model is in training mode.

    --- args ---
    block_size : int
        the (square) size of blocks that will be dropped.
    keep_prob : float between 0 and 1
        the probablility that a certain cell will be kept after going through
        this layer. this can be modified later by setting keep_prob.
    '''
    def __init__(self, block_dict):
        super(DropBlock2D, self).__init__() # run super init

        # save the input variables
        self.n = block_dict['layer_num']
        self.block_size = block_dict['block_size']
        self.target_keep_prob = block_dict['target_keep_prob']
        self.init_keep_prob = block_dict['init_keep_prob']
        self.keep_prob = block_dict['init_keep_prob']

    def set_kp(self, pct):
        ''' set keep prob method

        --- args ---
        pct : float between 0 and 1, optional (defualt=0)
            the percent through training that we are currently
        '''
        self.keep_prob = self.init_keep_prob + \
            pct * (self.target_keep_prob - self.init_keep_prob)
    
    def forward(self, x):
        ''' feed a batch x through the module 
        note that this module will only affect the output if it's in training 
        mode. if its in evaluation mode, it will just return the input.
        
        --- args ---
        x : torch.tensor
            the input to the layer.
        '''
        # +++ first, make sure we're training
        if self.training == False: # if we're not training DO NOTHING
            return x

        # +++ get info about the x tensor
        device = x.device
        batch_size, n_feats, height, width = x.size()

        # +++ since we are training, we need to make the mask and shit
        gamma = (1 - self.keep_prob) / self.block_size ** 2 # calculate the gamma value

        mask = torch.rand((batch_size, 1, height, width)) # random array
        mask = (mask < gamma).float() # keep only ones < gamma
        mask = mask.to(device) # send to same device as x

        # +++ now we need to make the mask into a block mask
        bmask = F.max_pool2d(
            input=mask,
            kernel_size=self.block_size,
            stride=1,
            padding=self.block_size//2)
        
        if self.block_size % 2 == 0: # if it's an even block size
            bmask = bmask[:,:,:-1,:-1] # sluff off a bit of the edges
        
        bmask = 1 - bmask # flip 1s and 0s

        # +++ apply the block mask to the input
        out = x * bmask

        # +++ scale the output
        out = out * (bmask.numel() / bmask.sum())

        return out

# +++ net blocks dictionary
NET_BLOCKS = {
    'convolutional': ConvolutionalBlock,
    'fire': FireBlock,
    'maxpool': MaxPoolBlock,
    'upsample': UpsampleBlock,
    'route': RouteBlock,
    'shortcut': ShortcutBlock,
    'dropblock': DropBlock2D}

# +++ TESTING CODE
if __name__ == '__main__':
    from datamanager.datamanager import DataManager
    classes = 12
    n_anchors = 3
    batch_size = 1

    block_dict = {
        'layer_num': 24,
        'type': 'detection',
        'name': 'det2-24',
        'anchors': [[12, 26], [14, 29], [15, 34]],
        'ignore_thresh': 0.7,
        'lambda_bbox': 5.0,
        'lambda_noobj': 0.5,
        'lambda_obj': 1.0,
        'lambda_cls': 1.0,
        'loss_reduction': 'sum',
        'input_dim': 256,
        'classes': 12,
        'lbl_smoothing': 0.05}
    
    dm = DataManager('test-data', 256)
    batches = dm.batches(batch_size=batch_size, shuffle=False)
    _, y = batches[5]
    y = y.cuda()
    x = torch.randn((batch_size, (5+classes)*n_anchors, 8, 8)).cuda()

    detb = DetBlock(block_dict, CUDA=True)
    o, l  = detb(x, targets=y)

    