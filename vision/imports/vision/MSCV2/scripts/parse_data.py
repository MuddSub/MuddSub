''' parse_data.py
This script just runs the data parser from the datamanager on a specified set
of data.

'''
from datamanager import DataParser

dp = DataParser('raw-data', 'class-names.txt', 256)
dp.save('data/train-data-256', 'data/test-data-256')
