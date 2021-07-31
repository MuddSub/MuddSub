import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import pickle
from Dataloader import *
from FastSLAM2 import FastSLAM2
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import argparse
from Util import wrap_to_pi
from Models import Meas, FastSLAM2Parameters
from RobotPhysics2D import RobotPhysics2D
from Validation import plot_data
ROBOT_ID = 0
START_STEP = 0
NUM_STEPS = 20000
MEAS_COV = np.diag([0.01, 0.01])
SENSOR_RANGE = 1
SENSOR_BEARING = np.pi
LANDMARK_NUM = None #14

HARDCODE_COMPASS = True
HARDCODE_MEAS = False
NO_MEASUREMENTS = False
PLOT_AVG = True
INIT_LANDMARKS = True
KNOWN_CORRESPONDENCES = True

'''
threshold: .3, meas cov: .1, motion cov: 1e-2
'''

plot_data_list = []
groundtruth_path_data = []
num_particles = 1
default_pose_cov = np.diag([1e-4, 1e-4, 1e-4])
default_land_cov = np.diag([1e-4, 1e-4])

random_generator = np.random.default_rng()
algorithm = None

def run_fast_slam2(pkl = '../../datasets/Jar/dataset1.pkl'):
  global algorithm
  dataloader = pickle.load(open(pkl,'rb'))
  robot_data = dataloader.robots[ROBOT_ID]

  # Create the initial map
  initial_landmarks = {}
  if INIT_LANDMARKS:
    for idx, landmark in dataloader.map.landmark_dict.items():
      land_mean = np.array([landmark['X'], landmark['Y']])
      initial_landmarks[idx] = (land_mean, default_land_cov)

  # Create FastSLAM2Parameters
  params = FastSLAM2Parameters(
    num_particles = num_particles,
    is_landmarks_fixed = True,
    initial_landmarks = initial_landmarks
  )

  # Create instance of FastSLAM2
  random = np.random.default_rng()
  initial_pose = np.array([robot_data.get_x_truth(0),robot_data.get_y_truth(0), robot_data.get_compass(0)])
  robot_physics = RobotPhysics2D(random, initial_pose, default_pose_cov)
  algorithm = FastSLAM2(robot_physics, parameters=params, random=random)

  # Start loop
  theta = 0
  for i in range(NUM_STEPS):
    
    theta_p = theta
    update = robot_data.get_next()
    
    t = update[1][0]
    groundtruth_path_data.append([robot_data.get_x_truth(t),robot_data.get_y_truth(t)])
    if i==0:
      algorithm.prev_t = t
      algorithm._robot_physics.initial_pose[2] = wrap_to_pi(robot_data.get_compass(t))
    if update[0] == "odometry":
      odometry = update[1]

      # Use groundtruth to calculate odometry input
      time, velocity, angular_velocity = odometry

      # Update particle poses
      algorithm.add_control((velocity, angular_velocity), t)

      # Hard coding poses
      if HARDCODE_COMPASS:
        for j in range(len(algorithm.particles)):
          x = algorithm.particles[j].pose[0]
          y = algorithm.particles[j].pose[1]
          theta = robot_data.get_compass(t)
          algorithm.particles[j].pose = np.array([x, y, theta])
    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement
      if not NO_MEASUREMENTS:
        # Update EKFs
        if (LANDMARK_NUM == None and subject > 5) or subject == LANDMARK_NUM: #if subject > 5 :
          landmark = dataloader.map.get_landmark_location(subject)
          landmark_x = landmark['X']
          landmark_y = landmark['Y']

          # Use groundtruth to provide accurate measurement
          if HARDCODE_MEAS:
            robot_x = robot_data.get_x_truth(time)
            robot_y = robot_data.get_y_truth(time)
            robot_angle = robot_data.get_compass(time)
            range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
            bearing_meas = wrap_to_pi(np.arctan2(landmark_y - robot_y, landmark_x - robot_x) - robot_angle)
          print("step", i, "updated measurement", subject, "with position", [landmark_x, landmark_y])
          
          meas = Meas((range_meas, bearing_meas), MEAS_COV,(SENSOR_RANGE, SENSOR_BEARING), subject)         
          algorithm.add_measurement(meas)
    
    # Log data
    _, *slam_snapshot = algorithm.get_pose_and_landmarks_for_plot()
    plot_data_list.append([i,*slam_snapshot])

  # Extract landmark groundtruth from dataloader
  landmarks_groundtruth = []
  for idx, landmark in dataloader.map.landmark_dict.items():
    if LANDMARK_NUM == None or idx == LANDMARK_NUM:
      landmarks_groundtruth.append(np.array([landmark['X'], landmark['Y']]))
  landmarks_groundtruth = np.array(landmarks_groundtruth)
  
  print("num landmark: ground truth", len(landmarks_groundtruth), " what we got", len(algorithm.particles[0].landmarks))

  plot_data(num_particles, plot_data_list,groundtruth_path_data, landmarks_groundtruth)
def error_ellipse(points, cov, nstd=2):
    """
    Source: http://stackoverflow.com/a/12321306/1391441
    """
    def eigsorted(cov):
      '''
      Eigenvalues and eigenvectors of the covariance matrix.
      '''
      vals, vecs = np.linalg.eigh(cov)
      order = vals.argsort()[::-1]
      return vals[order], vecs[:, order]

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[::-1, 0]))

    # Confidence level
    q = 2 * norm.cdf(nstd) - 1
    r2 = chi2.ppf(q, 2)

    width, height = 2 * np.sqrt(vals * r2)

    return width, height, theta


run_fast_slam2()