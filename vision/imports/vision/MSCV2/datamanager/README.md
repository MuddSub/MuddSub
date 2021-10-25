# Data Manager Package
This package manages the data i/o for an object-detection neural network. This readme will give a brief outline for how the module can be used, in roughly chronological order of how one might use it.

# Data Parsing
This section details the initial parsing of the data which is accomplished through the `DataParser` class.

## Data Formatting
The input data here should be a folder, for this example we'll call this folder `raw-data`, and it should contain images like `[filename].jpg` and label files (in YOLO format) that look like `[filename].txt`. There should also be a file like `class-names.txt` that had the name of each class on a new line, in order of their class number. Finally, you will need to have some square `resize_dim` that each input image will be resized to.

## Loading the Data
The first step in parsing data is loading the data into the `DataParser`. This is done when you initialize the parser like so:
```python
from datamanager import DataParser
dp = DataParser(raw_data_path='raw-data', class_file='class-names.txt', resize_dim)
```

## Trimming the Data
Sometimes the data contains bad datapoints. The `DataParser` class has a `trim_data` method that allows the user to flip through each datapoint one at a time and allows you to choose to keep the image by hitting space, or throw out the image by hitting any other key.

## Saving the Data
Once the data has been loaded and prepared, it can be saved to a pytorch .pt file for easy loading into the rest of the network framework. You'll need to decide what proportion of the data you would like to be set aside for testing and validation, `pct_test` which is given a value of 0.1 (10%) if not provided. Furthermore we need to have a name in mind for the output training and testing data files. This whole process might look like this
```python
dp.save(
    train_fn = 'train-data', # train data will be at 'train-data.pt'
    test_fn = 'test-data', # test data at 'test-data.pt'
    test_pct = 0.05 # 5% of data set aside for testing
)
```

# Loading the Data for the Network
When it comes time to load the data for testing the `DataManager` class is the one to care about. The data can be loaded like so
```python
from datamanager import DataManager
train_data = DataManager(
    data_path = 'train-data.pt',
    CUDA = use_cuda_for_data_aug,
    **aug_dict
)
```
(See _Data Augmentation_ section for more info on data augmentation)

Then, one can iterate through batches in one epoch like so:
```python
for batch in train_data.batches(
    batch_size = ...,
    mini_batch_size = ..., # (batch_size by default)
    ):
    ...
    for x, y in batch:
        ...
        # do feedforward/backprop
        ...
```

# Data Augmentation
Many methods of data augmentation methods and options are available through the `DataManager` including
* Data creation augmentation
    * `mosaics` - mosaic image creation
    * `mixup` - pixel value blending
    * `cutmix` - cutout image mixing
    * `cutout` - cutout removal
* Data manipulation augmentation
    * `hflip` - horizontal flipping
    * `vflip` - vertical flipping
    * `rot` - random rotation by a multiple of 90 deg
* Nitty gritty
    * `iou_thresh` - the iou thresholds for augmentation, by default `(0.7,0.2)`. When trimming, boxes will be dropped if the iou of the original with the portion inside the trim is greater than the first threshold value. When cropping, boxes will only be kept if the iou of the original with the cropped box exceeds the second threshold value.
    * `cut_range` - the minimum and maximum cut sizes respectively, by default `(0.3,0.5)`, both relative to the image size, during cutout-like data augmentation

All these values get placed in a dictionary, where each of the data augmentation values are floats between 0 and 1. For image creation, that proportion of the data set's images will be created. For manipulation, that proportion of the data sets images (including newly created ones) will have the manipulation applied to them.