''' parse_data.py
this file is used to load and split the data into training and testing 
directories. this file only needs to be run once, when you first upload the 
dataset you wish to use for the network.

dataset formatting: the dataset should come in one directory with image files 
(.jpg) and text files (.txt). the image files should look like [datapoint].jpg,
and the label files should look like [datapoint].txt. labels should be in the
YOLO format. there should also be a class file somewhere with the .txt extension
that gives names to the classes from the label files.
'''

############################## SET THESE VALUES ################################
data_in_dir = 'raw-data' # directory to get the raw data from
train_data_dir = 'train-data' # folder for training data
test_data_dir = 'test-data' # folder for test data
class_file = 'class-names.txt' # file for the class names
test_pct = .10 # percent of data that should be allocated for testing
shuffle = True # should the data be shuffled?
################################################################################

# +++ imports
import os
import random
from math import log10
from shutil import copyfile

# +++ function to make directories
def mkdir(*paths):
    # function to make directories and sub directories
    path = os.path.join(*paths)
    path = os.path.realpath(path)
    if not os.path.isdir(path):
        os.mkdir(path)
    return path

# +++ open the in data directory and get the files
data_in_dir = os.path.realpath(data_in_dir) # data directory path
files = os.listdir(data_in_dir) # get the file names in the path
file_paths = [os.path.join(data_in_dir, f) for f in files] # turn them into legit paths

# +++ make some directories and paths and shit
train_data_dir = mkdir(train_data_dir)
train_img_dir = mkdir(train_data_dir, 'images')
train_lbl_dir = mkdir(train_data_dir, 'labels')
# make test directory if the there is testing data
if test_pct != 0.:
    test_data_dir = mkdir(test_data_dir)
    test_img_dir = mkdir(test_data_dir, 'images')
    test_lbl_dir = mkdir(test_data_dir, 'labels')

# +++ now go through the files to build the data and shit
all_files = [] # list of files, in tuples (img_path, lbl_path)

# +++ loop to get the files for training and testing
while len(file_paths) > 0: # while there are still paths to go through
    # +++ get the file and info about it
    fp = file_paths.pop(0)
    fname = fp.split('/')[-1]
    fext = fname.split('.')[-1]

    # +++ cases for file extensions
    if fext == 'txt': # if it's a text file
        continue # skip over it (we'll get those with the images)
    elif fext != 'jpg': # if it isn't jpg
        # that's fine, just print a warning and skip it
        print(f'warning: skipping file with unknown extension \'{fname}\'')
        continue
    
    # +++ get the label filepath and make sure it exists
    img_fp = fp
    lbl_fp = img_fp[:-len(fext)] + 'txt'
    if not os.path.exists(lbl_fp):
        print(f'warning: could not find label for image \'{fname}\', skipping it')
        continue
    
    # +++ add the files to the list of files to copy
    all_files.append((img_fp, lbl_fp))

# +++ split the files into test files and train files
if shuffle: # if they should be shuffled
    random.shuffle(all_files) # shuffle em first
n_test = int(test_pct * len(all_files)) # calculate number of files for testing
test_files = all_files[:n_test] # allocate test files
train_files = all_files[n_test:] # allocate training files

# +++ loop to save the test images
num_len = int(log10(len(test_files)-1)) + 1 # length (characters) of the max number for this set
if n_test != 0: # if there were testing images
    for i, (img_path, lbl_path) in enumerate(test_files):
        img_to_path = os.path.join(test_img_dir, f'img{i:0>{num_len}}.jpg')
        lbl_to_path = os.path.join(test_lbl_dir, f'img{i:0>{num_len}}.txt')
        copyfile(img_path, img_to_path)
        copyfile(lbl_path, lbl_to_path)

# +++ loop to save the training images
num_len = int(log10(len(train_files)-1)) + 1 # length (characters) of the max number for this set
for i, (img_path, lbl_path) in enumerate(train_files):
    img_to_path = os.path.join(train_img_dir, f'img{i:0>{num_len}}.jpg')
    lbl_to_path = os.path.join(train_lbl_dir, f'img{i:0>{num_len}}.txt')
    copyfile(img_path, img_to_path)
    copyfile(lbl_path, lbl_to_path)

# +++ lastly, save a copy of the class names file to each directory
copyfile(class_file, os.path.join(test_data_dir, class_file))
copyfile(class_file, os.path.join(train_data_dir, class_file))
