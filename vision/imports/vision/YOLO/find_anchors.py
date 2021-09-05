''' find_anchors.py
this file is used to find the optimal anchor boxes for a particular network
architecture. it uses kmeans clustering to find the best shapes for the boxes
to be prior to yolo detection.
'''
import os
import argparse
import torch
from sklearn.cluster import KMeans
from datamanager import load_labels

if __name__ == '__main__':
    # +++ parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('data_path', type=str,
        help='the path to the raw data, where labels are all stored in a .txt format')
    parser.add_argument('input_dim', type=int,
        help='the input dimension of images to the network')
    parser.add_argument('num_anchors', type=int,
        help='the number of anchors that the network should have (total)')
    args = parser.parse_args() # get the args

    # +++ get all the files in the datapath
    data_path = os.path.realpath(args.data_path) # get the real path to the data
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
    whdata *= args.input_dim # make units pixels

    # +++ do kmeans
    kmeans = KMeans(n_clusters=args.num_anchors, n_init=10)
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




