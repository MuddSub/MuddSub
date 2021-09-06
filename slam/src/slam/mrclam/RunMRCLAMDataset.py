import warnings
import os
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)
import pickle
import pandas as pd
from slam.mrclam.MRCLAMDataloader import *
from slam.fast_slam2.FastSLAM2 import FastSLAM2
import numpy as np
from slam.Utils import wrap_to_pi, plot_df
from slam.fast_slam2.Models import Meas, FastSLAM2Parameters, LandmarkConstants
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D

class RunMRCLAMDataset():
  '''
  Runs FastSLAM on the MRCLAM Dataset.

  The MRCLAM dataset is a 2D, mutli-robot dataset. More details: http://asrl.utias.utoronto.ca/datasets/mrclam/index.html
  This is a runner for using MRCLAM with FastSLAM. It can take in different FastSLAM versions and different robot physics.  
  '''
  def __init__(self, **kwargs):
    '''
    Initializes the runner.

    Kwargs:
      hardcode_pose:          If True, the gorundtruth pose will be passed to the algorithm. Defaults to False.
      hardcode_compass:       If True, the groundtruth orientation (but not position) will be passed to the algorithm. Defaults to False.
      hardcode_meas:          If True, the groundtruth measurements will be passed to the algorithm. Defaults to False
      no_measurements:        If True, no measurements will be passed to the algorithm. Defaults to False.
      init_landmarks:         If True, the groundtruth map will initially be passed to the algorithm. Defaults to False
      robot_id:               Takes values between 0 and 4 for the 5 robot datasets. Defaults to 0.
      start_step:             The time step in the dataset to start at. Defaults to 0.
      num_steps:              The number of time steps of the dataset to run. Defaults to 1000.
      meas_cov:               The measurement covariance. Defaults to np.diag([0.075, 0.025]).
      sensor_range:           The range of the robot's sensor. Defaults to 100 which is greater than necessary since the physical sensor was a camera.
      sensor_fov:             The FOV of the robot's sensor. Defaults to pi.
      skipped_meas:           If True, any measurement that doesn't fall in the range and FOV is skipped. This can be used to simulate different sensor contraints. Defaults to False.
      known_correspondences:  If True, the obstacle's identity is provided to the algorithm as part of the measurement.
      verbose:                If True, messages will be printed by the algorithm. Defaults to False.
      fast_slam_version:      Takes the value 1 or 2, for FastSLAM1 or FastSLAM2 respectively. Defaults to 2.
      plot_msg:               The message to go with the plot. Defaults to ''.
      num_particles:          The number of particles to initialize the algorithm with. Defaults to 1.
      initial_pose_cov:       The initial pose covariance. Defaults to np.diag([1e-3, 1e-3, 1e-4]).
    '''
    self.hardcode_pose = kwargs.get('hardcode_pose', False)
    self.hardcode_compass = kwargs.get('hardcode_compass', False)
    self.hardcode_meas = kwargs.get('hardcode_meas', False)
    self.no_measurements = kwargs.get('no_measurements', False)
    self.init_landmarks = kwargs.get('init_landmarks', False)
    self.robot_id = kwargs.get('robot_id', 0)
    self.start_step = kwargs.get('start_step', 0)
    self.num_steps = kwargs.get('num_steps', 1000)
    self.meas_cov = kwargs.get('meas_cov', np.diag([.075,.025]))
    self.sensor_range = kwargs.get('sensor_range', 100)
    self.sensor_fov = kwargs.get('sensor_fov', np.pi)
    self.skipped_meas = kwargs.get('skipped_meas', False)
    self.known_correspondences = kwargs.get('known_correspondences', True)
    self.verbose = kwargs.get('verbose', False)
    self.fast_slam_version = kwargs.get('fast_slam_version', 2)
    self.plot_msg = kwargs.get('plot_msg', '')
    self.num_particles = kwargs.get('num_particles', 1)
    self.initial_pose_cov = kwargs.get('initial_pose_cov', np.diag([1e-3, 1e-3, 1e-4]))

    self.history = None
    self.groundtruth_path_data = []
    self.random_generator = np.random.default_rng()
    self.algorithm = None
    self.update = None

  def load_data(self, pkl):
    '''
    Loads the dataset from a pickle file.

    Args:
      pkl:  The path to the pickle file.
    '''
    global algorithm
    self.dataloader = pickle.load(open(pkl, 'rb'))
    self.robot_data = self.dataloader.robots[self.robot_id]
    if self.init_landmarks:
      self.params.initial_landmarks = {}
      for idx, landmark in self.dataloader.map.landmark_dict.items():
        x = landmark['X']
        y = landmark['Y']
        self.params.initial_landmarks[idx] = (np.array([x, y]), None)

  def run_fast_slam2(self):
    '''
    This is the main function with a control loop that calls all of the other functions.
    '''
    # Set up parameters needed to initialize the FastSLAM2 algorithm
    initial_pose = np.array([self.robot_data.get_x_truth(0), self.robot_data.get_y_truth(0), wrap_to_pi(self.robot_data.get_compass(0))])    
    self.params = FastSLAM2Parameters(
      num_particles = self.num_particles,
      are_landmarks_fixed = True,
      initial_landmarks = {},
      initial_pose = initial_pose,
      initial_pose_cov = self.initial_pose_cov,
      landmark_constants = LandmarkConstants(),
      localization_only = False,
      verbose = 2 if self.verbose else 0, 
      fast_slam_version = self.fast_slam_version,
    )
    random = np.random.default_rng()
    robot_physics = RobotPhysics2D(random)
    self.algorithm = FastSLAM2(robot_physics, parameters = self.params, random = random)

    # Main loop through the dataset
    theta = 0
    for i in range(self.num_steps):
      # Retrieve data from the dataset
      self._log('step', i)
      self.update = self.robot_data.get_next()
      t = self.update[1][0]
      self.groundtruth_path_data.append([self.robot_data.get_x_truth(t), self.robot_data.get_y_truth(t)])

      # Perform update based on the type of the data
      if i == 0:
        self.algorithm.prev_t = t
      if self.update[0] == "odometry":
        self._add_control(t)

        # Hard coding compass or pose
        if self.hardcode_pose or self.hardcode_compass:
          for j in range(len(self.algorithm.particles)):
            if self.hardcode_pose:
              x = self.robot_data.get_x_truth(t)
              y = self.robot_data.get_y_truth(t)
            else:
              x = self.algorithm.particles[j].pose[0]
              y = self.algorithm.particles[j].pose[1]
            theta = self.robot_data.get_compass(t)
            self.algorithm.particles[j].pose = np.array([x, y, theta])
      else:
        self._add_meas()
      self.log_data()

  def plot(self):
    '''
    Plots the algorithm's history after its run.
    '''
    landmarks_groundtruth = []
    for _, landmark in self.dataloader.map.landmark_dict.items():
      landmarks_groundtruth.append(np.array([landmark['X'], landmark['Y']]))
    landmarks_groundtruth = np.array(landmarks_groundtruth)
    print("num landmark: ground truth", len(landmarks_groundtruth), " what we got", len(self.algorithm.particles[0].landmarks))
    plot_df(self.history, self.groundtruth_path_data, landmarks_groundtruth, msg=self.plot_msg)
  
  def _log(self, *msg):
    '''
    Logs some messages.

    Args:
      *msg:   The list of messages to print.
    '''
    if self.verbose:
      print('Run MR.CLAM', *msg)
  
  def log_data(self):
    '''
    Add the current timestep's information to the history.
    '''
    snapshot = self.algorithm.get_pose_and_landmarks_for_plot()
    if self.history is None:
      self.history = snapshot
    else:
      self.history = pd.concat([self.history, snapshot], ignore_index=True)

  def _add_control(self, t):
    '''
    Pass the current control to the algorithm.
    '''
    odometry = self.update[1]

    # Use groundtruth to calculate odometry input
    time, velocity, angular_velocity = odometry

    # Update particle poses
    self.algorithm.add_control((velocity, angular_velocity), t)

  def _add_meas(self):
    '''
    Pass the current measurement to the algorithm.
    '''
    measurement = self.update[1]
    time, subject, range_meas, bearing_meas = measurement

    # Update EKFs
    if not self.no_measurements:
      if subject > 5:
        landmark = self.dataloader.map.get_landmark_location(subject)
        landmark_x = landmark['X']
        landmark_y = landmark['Y']

        # Use groundtruth to provide accurate measurement
        if self.hardcode_meas:
          robot_x = self.robot_data.get_x_truth(time)
          robot_y = self.robot_data.get_y_truth(time)
          robot_angle = self.robot_data.get_compass(time)
          range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
          bearing_meas = wrap_to_pi(np.arctan2(landmark_y - robot_y, landmark_x - robot_x) - robot_angle)

        # Check to see if the measurement should be skipped if its out of range
        if self.skipped_meas:
          if range_meas > self.sensor_range or abs(bearing_meas) > self.sensor_fov:
            return

        # Remove subject if correspondences are unknown
        if not self.known_correspondences:
          subject = None

        meas = Meas((range_meas, bearing_meas), self.meas_cov,  np.array([self.sensor_range, self.sensor_fov]), subject)
        self.algorithm.add_measurement(meas)

if __name__ == '__main__':
  clam_dataset = RunMRCLAMDataset(init_landmarks = False, num_particles = 2, num_steps = 10, hardcode_compass = True, verbose = True, plot_msg = 'Fast SLAM 2')
  clam_dataset.load_data(os.path.dirname(os.path.realpath(__file__)) + '/datasets/Jar/dataset1.pkl')
  clam_dataset.run_fast_slam2()
  clam_dataset.plot()
