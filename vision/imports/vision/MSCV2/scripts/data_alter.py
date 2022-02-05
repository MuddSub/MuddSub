''' data_alter.py
this file was written because i am a fuckup who didn't forsee the fact that one
might want to resize data after having gone through it with trim-data.py. so
that is what this file does: takes an already saved .pt data set and resizes
the images to make another .pt data set.
'''
# +++ imports
import os
import numpy as np
import torch
from datamanager import load_images, load_labels

# +++ PREFERENCES
PARENT_DIR = 'data'
RAW_DATA_DIR = 'raw-data'
IN_DATA_SET = 'train-data-256.pt'
NEW_DIM = 416
OUT_DATA_SET = 'train-data-416.pt'

# +++ get the actual file paths
in_fn = os.path.join(PARENT_DIR, 'test-data-t.pt')
out_fn = os.path.join(PARENT_DIR, f'test-data-t-{NEW_DIM}.pt')

# +++ open the input data set
f = open(in_fn, 'rb')
data, classes, dim = torch.load(f)
f.close()

# +++ get the file names
fnames = data[:,0].tolist()

# +++ make a new data set with the right dimensions
new_data = np.concatenate((
    load_images(RAW_DATA_DIR, fnames, NEW_DIM),
    load_labels(RAW_DATA_DIR, fnames)), axis=1)

# +++ save the new data set
torch.save((new_data, classes, NEW_DIM), out_fn)
