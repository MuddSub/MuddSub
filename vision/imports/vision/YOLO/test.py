
# from datamanager import DataParser
import random
from datamanager import *
import torch
import time
import copy

data_aug = {
    'mosaics': 0.25,
    'mixup': 0.25,
    'cutmix': 0.25,
    'cutout': 0.25,
    'hflip': 0.25,
    'vflip': 0.25,
    'rot': 0.25}

dm = DataManager('data/trimmed/train-data-416.pt', CUDA=True, **data_aug)

dm.premake_batches('TRD-evenaug-416', 1500, batch_size=64, mini_batch_size=32, shuffle=True, augmented=True)

# data_aug = {
#     'mosaics': 0.25,
#     'mixup': 0.25,
#     'cutmix': 0.25,
#     'cutout': 0.25,
#     'hflip': 0.25,
#     'vflip': 0.25,
#     'rot': 0.25
# }
# a = torch.zeros(10).cuda()
# del a

# dm = DataManager('data/train-data.pt', CUDA=True, **data_aug)

# tstart = time.time()
# batches = dm.batches(batch_size=12)
# tend = time.time()

# print(f'augmentation and batching took {tend-tstart:.5f} seconds')

# imgs = []
# lbls = []
# for mb in batches:
#     for x, y in mb:
#         for img in x:
#             imgs.append(to_cvUMat(img))
#         lbls += y

# ######### NEED TO MAKE BATCHES OF LABELS BE TENSORS!!!


# # cimgs = [to_cvUMat(img) for img in imgs]
# dimgs = dm.write_boxes(lbls, imgs)

# for _ in range(30):
#     i = random.randint(0, len(dimgs)-1)
#     show_img(dimgs[i])



'''
CUDA = True
imgs = dm.get_imgs()[:10]
lbls = dm.get_lbls()[:10]

oimgs, olbls = aug_funcs.make_cutout_aug(imgs, lbls, 10)

oimgs = [to_cvUMat(img) for img in oimgs]
dimgs = dm.write_boxes(olbls, images=oimgs)

for img in dimgs:
    show_img(img)
'''

'''
n = len(dm)

print(f'---------- WITH CUDA = {CUDA} ----------')

n = 600
imgs = imgs[:n]
lbls = lbls[:n]

if CUDA:
    imgs = imgs.cuda()
    lbls = [lbl.cuda() for lbl in lbls]

args1 = (imgs, lbls, n)
args2 = (imgs, lbls)

for func in [make_mosaics_aug, make_mixup_aug, make_cutmix_aug]:
    args = copy.deepcopy(args1)
    tstart = time.time()
    func(*args)
    tend = time.time()
    print(f'creating {n} datapoints with {func.__name__} took {tend-tstart:.5f} seconds')


for func in [cutout_aug, hflip_aug, vflip_aug, rot90_aug]:
    args = copy.deepcopy(args2)
    tstart = time.time()
    func(*args)
    tend = time.time()
    print(f'augmenting {n} datapoints with {func.__name__} took {tend-tstart:.5f} seconds')
'''


'''
########## BEFORE TRIM/CROP DATAS
---------- WITH CUDA = False ----------
creating 600 datapoints with make_mosaics_aug took 1.58133 seconds
creating 600 datapoints with make_mixup_aug took 0.59948 seconds
creating 600 datapoints with make_cutmix_aug took 0.83682 seconds
augmenting 600 datapoints with cutout_aug took 0.33843 seconds
augmenting 600 datapoints with hflip_aug took 0.54629 seconds
augmenting 600 datapoints with vflip_aug took 0.45712 seconds
augmenting 600 datapoints with rot90_aug took 0.47407 seconds

---------- WITH CUDA = True ----------
creating 600 datapoints with make_mosaics_aug took 0.76633 seconds
creating 600 datapoints with make_mixup_aug took 0.03700 seconds
creating 600 datapoints with make_cutmix_aug took 1.20461 seconds
augmenting 600 datapoints with cutout_aug took 0.52537 seconds
augmenting 600 datapoints with hflip_aug took 0.02721 seconds
augmenting 600 datapoints with vflip_aug took 0.03882 seconds
augmenting 600 datapoints with rot90_aug took 0.06716 seconds

########### AFTER TRIM/CROP DATAS
---------- WITH CUDA = False ----------
creating 600 datapoints with make_mosaics_aug took 1.31946 seconds
creating 600 datapoints with make_mixup_aug took 0.59990 seconds
creating 600 datapoints with make_cutmix_aug took 0.92556 seconds
augmenting 600 datapoints with cutout_aug took 0.33150 seconds
augmenting 600 datapoints with hflip_aug took 0.44015 seconds
augmenting 600 datapoints with vflip_aug took 0.44647 seconds
augmenting 600 datapoints with rot90_aug took 0.47178 seconds

---------- WITH CUDA = True ----------
creating 600 datapoints with make_mosaics_aug took 0.75790 seconds
creating 600 datapoints with make_mixup_aug took 0.03964 seconds
creating 600 datapoints with make_cutmix_aug took 1.10534 seconds
augmenting 600 datapoints with cutout_aug took 0.50531 seconds
augmenting 600 datapoints with hflip_aug took 0.02821 seconds
augmenting 600 datapoints with vflip_aug took 0.04891 seconds
augmenting 600 datapoints with rot90_aug took 0.06776 seconds
'''
