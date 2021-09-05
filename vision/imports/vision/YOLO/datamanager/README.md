# Data Manager Module
This module manages the data i/o for an object-detection neural network. This readme will give a brief outline for how the module can be used, in roughly chronological order of how one might use it.

## Importing from data manager
Once you place the datamanager folder in your working directory, you can import the module (or parts of it) like so:
```python
import datamanager # import the whole module
from datamanager import DataManager # import a single class
```

## Parsing data
The data should come in the form of a folder with image files ([filename].jpg) and label files ([filename].txt) mixed together, along with a class-names file that contains the classes we're detecting, in order. The next step then is to parse the data, putting all the info about it into saved .pt file(s) for easy reloading during training and testing. The `DataParser` class can be used for this. If the data resides in a folder called `raw-data`, the following code could be used to parse it into training/testing sets.
```python
from datamanager import DataParser
dp = DataParser(raw_data_path='raw-data', class_file='class-names.txt', resize_dim)
```
