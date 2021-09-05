
import torch
import torch.nn as nn
import torch.nn.functional as F
import argparse
from config import read_cfg

# +++ FUNCTIONS
    # for calculating the output size of each layer based on that layer's type,
    # parameters, and the output size from the layer before it

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

def convolution(insize, filters, kernel_size, stride=1, padding=0):
    '''
    this calculates the size of the output tensor when put through a convolution
    that has the parameters given.
    '''
    N, c, y, x = insize # unpack the input size
    ky, kx = make_tup(kernel_size) # get the kernel dimensions
    sy, sx = make_tup(stride) # get the stride for each dimension

    py, px = make_tup(padding) # get the padding dimensions
    py *= 2 # total y padding = 2 * padding per side
    px *= 2 # same for x
    
    y_out = (y + py - ky) // sy + 1 # calculate output y dimension
    x_out = (x + px - kx) // sx + 1 # calculate output x dim
    
    # return the output size
    return (N, filters, y_out, x_out)

def fire(insize, fsqueeze, fexpand):
    return (insize[0], fexpand*2, *insize[2:])

def maxpool(insize, kernel_size, stride=None, padding=0):
    '''
    calculates the output tensor size after being run through a maxpool layer
    with the same parameters.
    '''
    N, c, y, x = insize # unpack the input size
    ky, kx = make_tup(kernel_size) # get the kernel size
    
    # set / determine the stride
    if stride == None: # if no stride is given
        sy, sx = ky, kx # use the kernel size
    else: # otherwise
        sy, sx = make_tup(stride) # use the stride given
    
    # set / determine the padding
    py1, py2, px1, px2 = make_tup(padding, tup_len=4, expand_method=1) 
        # get padding per side
    py = py1 + py2 # total y padding
    px = px1 + px2 # total x padding
    
    y_out = (y + py - ky) // sy + 1 # calculate y output
    x_out = (x + py - ky) // sx + 1 # calculate x output

    # return the output size
    return (N, c, y_out, x_out)

def upsample(insize, scale_factor):
    '''
    calculates the output size of a tensor after being passed through an 
    upsampling layer with the same parameters.
    '''
    N, c, y, x = insize # unpack the input size
    sfy, sfx = make_tup(scale_factor) # get the scale_factor for each dimensions
    out_y = y * sfy # calculate the output y size
    out_x = x * sfx # calculate the output x size
    # return the output size
    return (N, c, out_y, out_x)

def route(*layer_sizes):
    '''
    calculates the output size of a tensor from a routing layer that routes 
    together tensors with the given sizes.
    '''
    out_size = None

    for i, lsize in enumerate(layer_sizes): # loop through layer sizes
        
        if out_size == None: # if no output yet
            out_size = lsize # set the output to be that size
            continue # and go on to the next layer

        lN, lc, ly, lx = lsize # unpack the layer size
        N, c, y, x = out_size # unpack the output size

        # error checking
        if lN != N:
            raise AttributeError(f'layer #{i} does not have the same batch size as previous ones')
        if ly != y:
            raise AttributeError(f'layer #{i} does not have the same y dimension as previous ones')
        if lx != x:
            raise AttributeError(f'layer #{i} does not have the same x dimension as previous ones')

        # set the output size
        out_size = (N, c + lc, y, x)
    
    return out_size

def shortcut(curr_size, shortcut_size):
    if curr_size != shortcut_size:
        raise AttributeError(f'current size {curr_size} does not match shortcut size {shortcut_size}')
    return shortcut_size

def format_args(args):
    out = '(' # output string
    for i, arg in enumerate(args):
        val = args[arg] # get the value of the argument
        out += f'{arg}={val}' # add the argument to output
        if i != len(args) - 1: # if it's not the last arg
            out += ', ' # add comma and space
        else: # if it IS the last arg
            out += ')' # add close parenthesis
    return out

# +++ main code
if __name__ == '__main__':
    # +++ parse the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('cfg_path',
        help='the relative path to the config (.cfg) file to test')
    parser.add_argument('--batch_size', dest='batch_size', default=32,
        type=int, help='the batch size (default: 32)')
    parser.add_argument('--stop_at', dest='stop_at', default=None,
        type=int, help='after which layer to stop the testing loop, if desired')
    args = parser.parse_args()
    
    # +++ read the config file
    blocks = read_cfg(args.cfg_path) # read the config file
    net_block = blocks[0] # get the network block

    # +++ get the input size to the network
    xsize = (
        args.batch_size, 
        net_block['channels'], 
        net_block['input_dim'], 
        net_block['input_dim'])
    layer_output_sizes = {-1: xsize} # dictionary for layer output sizes
    det_lns = [] # detection layer numbers
    det_grids = [] # detection grid sizes

    print(f'INPUT TENSOR\n    => {xsize}')
    for n, block in enumerate(blocks[1:]):
        btype = block['type'] # get the block's type

        if btype == 'convolutional':
            # setup layer args
            layer_args = {
                'filters': block['filters'],
                'kernel_size': block['kernel_size'],
                'stride': block['stride'],
                'padding': block['padding']}
            # get new size
            xsize = convolution(xsize, **layer_args)
        elif btype == 'fire':
            layer_args = {
                'fsqueeze': block['fsqueeze'],
                'fexpand': block['fexpand']}
            xsize = fire(xsize, **layer_args)
        elif btype == 'maxpool':
            # setup the layer args
            layer_args = {
                'kernel_size': block['kernel_size'],
                'stride': block['stride'],
                'padding': block['padding']}
            # get the new size
            xsize = maxpool(xsize, **layer_args)
        elif btype == 'upsample':
            # setup layer args
            layer_args = {'scale_factor': block['scale_factor']}
            # get the new size
            xsize = upsample(xsize, **layer_args)
        elif btype == 'shortcut':
            # get the layer it's from
            ln = block['from']
            if ln < 0:
                ln += n
            # setup layer args
            layer_args = {'from': ln}
            # get new size
            xsize = shortcut(xsize, layer_output_sizes[ln])
        elif btype == 'route':
            # get layers to route from
            lns = block['layers']
            if isinstance(lns, int):
                lns = [lns]
            lns = [ln + n if ln < 0 else ln for ln in lns]
            sizes = [layer_output_sizes[ln] for ln in lns]
            # setup layer args
            layer_args = {'from': lns}
            # get the new size
            xsize = route(*sizes)
        elif btype == 'dropblock':
            layer_args = {
                'block_size': block['block_size'],
                'target_keep_prob': block['target_keep_prob'],
                'init_keep_prob': block['init_keep_prob']}
        elif btype == 'detection':
            det_lns.append(n)
            det_grids.append(xsize[2:])
            layer_args = {
                'anchors': block['anchors']}
        
        layer_output_sizes[n] = xsize
        name = block['name'] # get the block's name
        if name == None: 
            print(f'{n} : {btype}') # print layer # and type
        else:
            print(f'{n} : {btype} - \"{name}\"')
        if layer_args != None: # if there are layer args
            for arg in layer_args:
                print(f'    {arg}: {layer_args[arg]}')
        print(f'=> {xsize}') # print output size

        if n == args.stop_at:
            print(f'testing stopped after layer {n}')
            break
    det_lns = str(det_lns)[1:-1] # detection layers string
    det_grids = str(det_grids)[1:-1] # detection grids as a string
    print(f'detection at layers : {det_lns}')
    print(f'with grid sizes : {det_grids}')