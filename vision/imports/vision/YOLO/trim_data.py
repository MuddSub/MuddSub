''' trim_data.py
this file can be run to allow a user to flip through the images in the raw data
one-by-one, selecting to keep or remove each image. since the documentation is
crappy, here is how it works:
1) setup the variables under the comment # +++ PREFERENCES
2) run this file
    a window should appear with an image, and the terminal should open with a
    number (the index of the image).
3) decide to keep or delete each image
    if you wish to keep the image, just hit enter. if you wish to delete an
    image, type anything (besides spaces or tabs or newlines) and then hit
    enter.
once you are done the data will be saved to your specified output file.
'''
# +++ imports
from datamanager import DataParser


# +++ PREFERENCES
RAW_DATA_PATH = 'raw-data'
CLASS_NAMES_PATH = 'class-names.txt'
RESIZE_DIM = 256
OUTPUT_TRAINING_DATA = 'data/train-data-t'
OUTPUT_TEST_DATA = 'data/test-data-t'
TEST_PERCENTAGE = 0.1


# +++ actual code
dp = DataParser(RAW_DATA_PATH, CLASS_NAMES_PATH, RESIZE_DIM)
dp.trim_data()
dp.save(OUTPUT_TRAINING_DATA, OUTPUT_TEST_DATA, test_pct=TEST_PERCENTAGE)
