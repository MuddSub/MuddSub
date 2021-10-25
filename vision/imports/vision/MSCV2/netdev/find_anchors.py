''' find_anchors.py
this file is used to find the optimal anchor boxes for a particular network
architecture. it uses kmeans clustering to find the best shapes for the boxes
to be prior to yolo detection.
'''
import os
import torch
from sklearn.cluster import KMeans
from vision.MSCV2.datamanager import load_labels

def find_anchors(data_path, input_dim, num_anchors):
    ''' find_anchors function
    Find the ideal anchors for a given dataset.
    
    Parameters
    ----------
    data_path : str
        String path to the data set to test.
    input_dim : int
        The input dimension of the network being used.
    num_anchors : int
        The number of anchor boxes that should be calculated.
    '''
    # +++ get all the files in the datapath
    data_path = os.path.realpath(data_path) # get the real path to the data
    file_names = os.listdir(data_path) # get all the files in the directory
    
    # +++ loop to get the all the labels
    label_names = []
    for fname in file_names:
        ### make sure this file is a label
        ext = fname.split('.')[-1]
        if ext != 'txt': # if it's not a text file
            continue # skip it
        else:
            label_names.append(fname[:-1-len(ext)])
    
    all_labels = list(load_labels(data_path, label_names).reshape(-1))
    all_labels = torch.cat(all_labels, dim=0) # stack together
    whdata = all_labels[:,-2:] # width and height data
    whdata *= input_dim # make units pixels

    # +++ do kmeans
    kmeans = KMeans(n_clusters=num_anchors, n_init=10)
    kmeans.fit(whdata) # fit kmeans to the data
    anchors = kmeans.cluster_centers_ # get anchors from kmeans

    # +++ format the anchor boxes
    anchors = torch.from_numpy(anchors).int() # make torch int tensor
    areas = anchors.prod(1) # get the areas of each anchor box
    sorted_idxs = torch.sort(areas)[1] # sort em by area
    anchors = anchors[sorted_idxs] # sort anchors

    # +++ format the output string
    outstr = ''
    for anch in anchors:
        outstr += f'{anch[0]},{anch[1]}  '
    outstr = outstr[:-2]

    print('--- calculated best anchors ---')
    print(outstr)




