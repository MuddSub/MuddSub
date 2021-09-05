
from datamanager import DataParser

dp = DataParser('raw-data', 'class-names.txt', 256)
dp.save('data/train-data-256', 'data/test-data-256')
