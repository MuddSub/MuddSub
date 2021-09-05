'''
reads config files
'''

# +++ GLOBAL VARIABLES
# special values dict : if any of these values are encontered, they will be
    # replaced by the values in the dict
SPECIAL_VALS = {
    'True': True,
    'False': False,
    'None': None}
# rename args dict : for each layer type, certain arguments will be renamed to
    # more accurate names. the name to be renamed to are keys, and the potential
    # given names are in the lists
RENAME_ARGS = {
    'net': {},
    'convolutional': {
        'size': 'kernel_size',
        'pad': 'padding'},
    'fire': {
        'fs': 'fsqueeze',
        'fe': 'fexpand'},
    'maxpool': {
        'size': 'kernel_size',
        'pad': 'padding'},
    'upsample': {
        'stride': 'scale_factor'},
    'route': {
        'from': 'layers'},
    'shortcut': {},
    'dropblock': {
        'keep_prob': 'target_keep_prob'},
    'conv-pre-yolo': {},
    'conv-pre-det': {},
    'detection': {}}
# default args dict : for each layer type, certain default arguments will be set
    # even if they are not explicitly given. if the default arg is None, then not
    # including the argument in the config file will throw an error
DEFAULT_ARGS = {
    'net': {
        'input_dim': None,
        'channels': 3,
        'classes': None},
    'convolutional': {
        'batch_normalize': False,
        'filters': None,
        'kernel_size': None,
        'stride': None,
        'padding': None,
        'activation': 'linear',
        'bias': True},
    'fire' : {
        'batch_normalize': False,
        'fsqueeze': None,
        'fexpand': None, 
        'activation': 'relu'},
    'maxpool': {
        'kernel_size': None,
        'stride': None,
        'padding': 0},
    'upsample': {
        'scale_factor': None},
    'route': {
        'layers': None},
    'shortcut': {
        'from': None,
        'activation': 'linear'},
    'dropblock': {
        'block_size': None,
        'target_keep_prob': None,
        'init_keep_prob': 1.},
    'conv-pre-yolo': {
        'batch_normalize': False,
        'kernel_size': 1,
        'stride': 1,
        'padding': 0,
        'activation': 'linear',
        'bias': True},
    'conv-pre-det': {
        'batch_normalize': False,
        'kernel_size': 3,
        'stride': 1,
        'padding': 1,
        'activation': 'linear',
        'bias': True},
    'detection': {
        'anchors': None,
        'mask': None,
        'ignore_thresh': 0.7,
        'lambda_obj': 1.,
        'lambda_noobj': 100.,
        'lambda_bbox': 1.,
        'lambda_cls': 1.,
        'loss_reduction': 'sum',
        'lbl_smoothing': 0.05}}


# +++ functions

def parse_value(value):
    ''' +++ INTERNAL FUNCTION +++ 
    takes in a string value and parses the value and datatype that it's meant 
    to encode.
    '''
    # +++ before anything, strip the value
    value = value.strip()
    # +++ make sure it's not nothing
    if value == '':
        return None # no value here
    # +++ check if it's NOT a list of any sort
    elif not ' ' in value and not ',' in value:
        # so it must be a number, string, or a special value
        try:
            if '.' in value:
                out = float(value)
            else:
                out = int(value)
            return out
        except ValueError:
            # if that throws a value error, it must be a string or special value
            if value in SPECIAL_VALS: # if it's a special value
                return SPECIAL_VALS[value] # return the value
            else: # if it's not
                return value # return the string
    # +++ otherwise it must be a list
    elif ' ' in value: # then it must be a list
        out = value.split(' ') # split by spaces
        out = [val for val in out if val != ''] # remove empty strings
        out = [parse_value(val) for val in out] # parse arguments individually
    elif ',' in value: # then it must be a list
        out = value.split(',') # split by spaces
        out = [val for val in out if val != ''] # remove empty strings
        out = [parse_value(val) for val in out] # parse arguments individually
    
    # so we had a list, now we make sure it's not a stupid list like [1] or []
    if len(out) == 0:
        return None
    elif len(out) == 1:
        return out[0]
    else:
        return out

def read_cfg(configFile):
    '''
    this function takes in a path to a .cfg file as described in DOCS/cfg-files
    and outputs a list of "network blocks" (essentially layers) that the network
    should have in it. the network blocks are dictionaries with attributes saved
    as keys.

    ---args---
    configFile: str
        the path to the configuration file that should be used to build this
        network.
    
    ---returns---
    list[dict]: the list of dictionaries describing each block of the network.
    '''
    # +++ read config file and pre-process a bit
    f = open(configFile,'r') # open the config file
    ftxt = f.read() # read the raw text of the file
    flines = ftxt.split('\n') # break it into a list of lines
    
    # +++ prepare some stuff for the loop
    networkBlocks = [] # list to hold all the blocks (~layers~(kinda)~)
    networkBlock = {} # the current network block we are working on

    # +++ main loop to build the initial network blocks
    for x in flines: # iterate through the lines of the file
        x = x.split('#') # split the line by comment mark
        x = x[0] # use only the part of the line before the comment
        x = x.strip() # strip it of spaces or whatever

        if len(x) == 0: # blank line
            continue # skip it
        
        elif x[0] == '[': # start of a new block
            if len(networkBlock) != 0: # if we are already working on a network block
                # add the old network block to the list of blocks
                networkBlocks.append(networkBlock)
                # and start a new block with the right layer index
                networkBlock = {'layer_num': len(networkBlocks) - 1}
            
            # set the type for the current network block to be this layer's type
            networkBlock['type'] = x.strip(' []')

        else: # if this is *not* a new network block
            attr, value = x.split('=') # get the attribute/value pair
            attr = attr.strip() # strip the attribute
            value = parse_value(value) # parse the value
            networkBlock[attr] = value # add them to the current block
    
    # add the last block we were working on to the list
    networkBlocks.append(networkBlock)
    
    # get some basic info about the network
    input_dim = networkBlocks[0]['input_dim'] # get the input dimension
    in_channels = networkBlocks[0]['channels'] # get the number of input channels
    num_classes = networkBlocks[0]['classes'] # get the number of classes

    # setup some shit for the next loop
    networkBlocks_output = [] # new list of network blocks to output
    output_channels = [in_channels] # holds the number of channels for each layer

    # +++ secondary loop to proccess the arguments of each block
    for i, block in enumerate(networkBlocks):
        btype = block['type'] # get the block type
        # +++ rename arguments
        rename_args = RENAME_ARGS[btype] # get the args to rename
        # reconstruct the block, updating the names if necessary
        block = {(rename_args[k] if k in rename_args else k):v 
                 for k,v in block.items()}
        
        # +++ set the default args
        block_args = DEFAULT_ARGS[btype] # get the default args for the block

        # set the name of the block (None/string)
        if not 'name' in block:
            block['name'] = None # set it to None
        else:
            block['name'] = str(block['name']) # make sure it's a string
        
        # loop to set defaults
        for arg in block_args:
            default = block_args[arg] # get the default value
            
            ### cases for default arg substitution
            if arg in block: # the block already has the argument
                pass # do nothing
            elif default != None: # block doesnt have arg and there is default
                block[arg] = default # set the arg in the block
            else: # otherwise... error
                raise AttributeError(f'block #{block["layer_num"]} missing required argument {arg}')
        
        # +++ apply the anchor mask for the yolo layers and add attrs
        if btype == 'detection':
            # add some attributes
            block['input_dim'] = input_dim # add input dimension attribute
            block['classes'] = num_classes # add classes attribute

            # apply anchor mask
            anchors = block['anchors'] # get all anchors
            mask = block['mask'] # get anchor mask
            block['anchors'] = [anchors[i] for i in mask] # set the anchors
            del block['mask'] # delete the mask
        
        # +++ special things to do with refrences to other layers
        if btype == 'route': # route layer
            n = block['layer_num'] # get the layer number
            layers = block['layers'] # get the layers it's routing from
            if type(layers) == int: # if only one layer was given
                layers = [layers] # make it a list
            layers = [l + n if l < 0 else l for l in layers] # make layer numbers absolute
            block['layers'] = layers # set the layers argument of the block        
        elif btype == 'shortcut': # shortcut layer
            n = block['layer_num'] # get the layer number
            l = block['from'] # get where it's coming from
            if l < 0: # if it's relative
                l += n # make it absolute
            block['from'] = l # update the from attribute
        
        # +++ pre yolo/pre det layer
        if btype in ['conv-pre-yolo', 'conv-pre-det']:
            detblock = networkBlocks[i+1] # get the detection block
            num_anchors = len(detblock['mask'])
            filters = (num_classes + 5) * num_anchors # filters for conv
            # update the block!
            block['type'] = 'convolutional' # type = convolutional
            block['filters'] = filters # set filters

        # +++ append the channels to the output_channels list
        if block['type'] == 'convolutional':
            block['in_channels'] = output_channels[-1] # input channels
            output_channels.append(block['filters']) # update filters
        elif block['type'] == 'fire':
            block['in_channels'] = output_channels[-1] # input channels
            output_channels.append(block['fexpand'] * 2) # output channels
        elif block['type'] == 'route':
            # calculate output channels
            channels = sum([output_channels[l+1] for l in block['layers']])
            # add to list
            output_channels.append(channels)
        # layers that don't affect the channels
        elif block['type'] in ['maxpool', 'upsample', 'shortcut', 'detection', 'dropblock']:
            output_channels.append(output_channels[-1]) # add the previous one again


        # +++ add the block to the output
        networkBlocks_output.append(block)
    
    # return the list of network blocks
    return networkBlocks_output
