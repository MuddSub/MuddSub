 import warnings
import os
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)
import pickle
import pandas as pd
from slam.mrclam.RunMRCLAMDataset import *

if __name__ == '__main__':
    clam_dataset = RunMRCLAMDataset(init_landmarks = False, num_particles = 1, num_steps = 20000, hardcode_compass = True, verbose = True, fast_slam_version = 1, plot_msg = 'Fast SLAM 1')
    clam_dataset.load_data(os.path.dirname(os.path.realpath(__file__)) + '/datasets/Jar/dataset1.pkl')
    clam_dataset.run_fast_slam2()
    clam_dataset.plot()
