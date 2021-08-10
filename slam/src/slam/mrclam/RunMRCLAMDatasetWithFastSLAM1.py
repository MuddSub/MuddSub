import warnings
import os
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)
import pickle
import pandas as pd
import numpy as np
from slam.mrclam.RunMRCLAMDataset import *

if __name__ == '__main__':
    clam_dataset = RunMRCLAMDataset(init_landmarks = False, num_particles = 20, num_steps = 20000, 
          hardcode_compass = True, verbose = True, fast_slam_version = 1, 
          plot_msg = 'Fast SLAM 1', initial_pose_cov = np.diag([1e-4, 1e-4, 1e-7]))
    clam_dataset.load_data(os.path.dirname(os.path.realpath(__file__)) + '/datasets/Jar/dataset1.pkl')
    clam_dataset.run_fast_slam2()
    clam_dataset.plot()
