''' yolo_model.py
this file contains an arbitrary model of a yolo network (yolo_net) that can be 
initialized with any yolo.cfg file and yolo.weights file. this allows for the
network's hyperparameters - even down to the layer by layer construction - to
be changed on the fly.

last updated 01/09/2021
'''

import torch 
import torch.nn as nn
import copy

# from util import * # import utility functions
# from util import *
from config import read_cfg # import function to read config file
from network_blocks import NET_BLOCKS, DetBlock # import network blocks

class NetworkModel(nn.Module):
    ''' network model
    this is the main class representing a network. depending on the config file
    it is fed, this may be a YOLO, SqueezeDet, or some kind of hybrid network.
    
    ---args---
    cfgfile : str
        the path to the configuration file to build this network
    CUDA : bool, optional (default=False)
        if true, the network will be run on the GPU.
    '''
    def __init__(self, cfgfile, CUDA=False):
        super(NetworkModel, self).__init__() # super init

        # +++ save cuda
        self.CUDA = CUDA # should cuda be used

        # +++ setup the network
        self.reset_metrics() # setup the metric dictionary
        self.netblocks = read_cfg(cfgfile) # read config file
        self._build_network() # build the network from the config file

    def _build_network(self):
        ''' +++ INTERNAL METHOD +++
        gets called by __init__ to build the network'''
        # +++ first, get some info about the network
        self.net_info = self.netblocks.pop(0) # get the network block
        self.input_dim = self.net_info['input_dim'] # img input dim
        self.num_classes = self.net_info['classes'] # number of classes
        self.num_blocks = len(self.netblocks) # set number of blocks

        # +++ get ready to build the network
        self.all_modules = nn.ModuleList() # list of blocks
        self.det_layers = [] # list of detection layers
            # this is needed to get the metrics later
        self.dropblock_layers = [] # list of dropblock layers
            # this is needed to set the keep percentage later

        # +++ main loop to build network
        for block in self.netblocks: # loop through network blocks

            btype = block['type'] # get the block's type

            if btype == 'detection':
                # setup the detection block
                block_module = DetBlock(block, CUDA=self.CUDA)
            else:
                # all other blocks can be initialized with the dict
                block_module = NET_BLOCKS[btype](block)

            # some special cases
            if btype == 'detection':
                self.det_layers.append(block_module)
            if btype == 'dropblock':
                self.dropblock_layers.append(block_module)

            self.all_modules.append(block_module) # add it to the list

    def set_db_kp(self, pct):
        ''' set drop block keep percentage method
        sets the keep percentage for each dropblock layer. using a linear model.
        
        --- args ---
        pct : float between 0 and 1
            the percentage through training. if 0, keep percentages will be set
            to the init value, if 1, the target value.
        '''
        for db in self.dropblock_layers: # go through dropblock layers
            db.set_kp(pct) # set the keep prob for each of them

    def forward(self, x, targets=None):
        '''
        forward function to feed a minibatch through the network
        
        ---args---
        x: torch.Tensor
            the minibatch; should have shape (batch_size, h, w, channels)
        '''
        # +++ setup variables before the loop
        layer_outputs = {} # outputs of each network layer (need to save each
            # since routing/shortcut layers will refer back to them)
        output = None # output variable (will store a tensor)
        loss = None # loss (will hold tensor)

        # iterate throught each module 
        for module in self.all_modules:
            
            # feed through the module (cases)
            if module.type == 'shortcut':
                # feed the current x and the shortcutting layer through
                x = module(x, layer_outputs[module.layer])
            elif module.type == 'route':
                # feed the layer outputs through
                x = module(layer_outputs)
            elif module.type == 'detection':
                # get the output and loss
                layer_out, layer_loss = module(x, targets=targets)

                # write output
                if output == None:
                    output = layer_out # set it to the current output
                else:
                    # concatenate with the current output
                    output = torch.cat((output, layer_out), dim=1)

                # write loss
                if targets != None:
                    if loss == None:
                        loss = layer_loss # set it to layer loss
                    else:
                        loss += layer_loss # add layer loss
                
            else: # all other layer types
                x = module(x) # just feed x through the module

            # save the output
            layer_outputs[module.n] = x
        
        # update the metrics
        if targets != None:
            self._update_metrics()
        
        if targets == None:
            return output # just return the output
        else:
            return output, loss # return output and loss

    def reset_metrics(self):
        ''' reset metrics method
        resets all of the current metrics to be zero.
        '''
        self._metrics = {
            'loss': 0.,
            'bbox-loss': 0.,
            'noobj-conf-loss': 0.,
            'obj-conf-loss': 0.,
            'conf-loss': 0.,
            'cls-loss': 0.,
            'cls-accuracy': 0.,
            'cls-acc-by-cls': torch.zeros(self.num_classes),
            'recall50': 0.,
            'recall75': 0.,
            'precision': 0.,
            'conf-obj': 0.,
            'conf-noobj': 0.}
        self.metric_batch_counter = 0.

    def _update_metrics(self):
        ''' +++ INTERNAL METHOD +++
        this method goes through all detection layer metrics and updates the
        class-wide metrics with them.
        '''
        ndet_layers = len(self.det_layers) # number of detection layers

        for detlayer in self.det_layers:
            for key in self._metrics:
                if key in detlayer.metrics:
                    # add the metric, dividing by the total number of detection layers
                    self._metrics[key] += detlayer.metrics[key] / ndet_layers
        
        self.metric_batch_counter += 1 # increment the batch counter

    @property
    def metrics(self):
        ''' metrics
        gets the metrics from the model, averaged over each batch and detection
        layer.
        '''
        batch_cnt = max(self.metric_batch_counter, 1.) # batch count
        metrics = copy.deepcopy(self._metrics)  # copy of the metrics dictionary
        for key in metrics: # go through metrics
            # average all metrics over the batches
            metrics[key] = metrics[key] / batch_cnt
        return metrics

    def print_metrics(self, classes=None, prefix='\t'):
        ''' print metrics method
        prints to the console the current metrics that the network has stored.
        
        --- args ---
        classes : list[str], optional (default=None)
            a list of the string names of the classes. if provided, class names
            will be printed instead of numbers.
        prefix : str, optional (defualt='\t')
            a prefix that will be attached to the start of everything printed.
        '''
        # +++ setup
        metrics = self.metrics()
        if classes is None:
            classes = [str(i) for i in range(self.num_classes)]
        
        # +++ loop to print
        for metric in metrics:
            if metric == 'cls-acc-by-cls':
                print(f'{prefix}{metric}:')
                for i in range(self.num_classes):
                    print(f'{prefix}\t({i}) {classes[i]}: {metrics[metric][i]:.4f}')
            else:
                print(f'{prefix}{metric}: {metrics[metric]:.4f}')
