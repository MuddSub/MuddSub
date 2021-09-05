''' misc.py
this file contains functions like show_img

'''
import torch
import numpy as np
import matplotlib.pyplot as plt
import cv2

def show_img(imgarr):
    if isinstance(imgarr, torch.Tensor):
        # input is a torch tensor
        img = imgarr.clone().permute(1,2,0).numpy()
    elif isinstance(imgarr, np.ndarray):
        # input is a cv2 array
        img = np.flip(imgarr, axis=2)
    plt.imshow(img)
    plt.show()


def to_cvUMat(tensor):
    arr = tensor.cpu().clone()
    arr = arr.flip(0).permute(1,2,0)
    # arr -= arr.min()
    # arr /= arr.max()
    arr *= 255
    arr = np.array(arr, dtype=np.uint8)
    arr = cv2.UMat(arr).get()
    return arr
