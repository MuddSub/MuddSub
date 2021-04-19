import pickle
from Dataloader import *
import FastSLAM2
import numpy as np

import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
ROBOT_ID = 0
NUM_STEPS = 10000
MEAS_COV = np.diag([0.075, 0.025])
params = {}
params['initial_pose'] = np.array([0, 0, 0, 0, 0, 0, 0])
params['num_landmarks'] = 0
params['v_sigma'] = 0.04
params['omega_sigma'] = 0.05
params['theta_sigma'] = 0.0125
params['prob_threshold'] = .5
params['sensor_range'] = 10
#TODO Figure out how to get variance for x and y
params['x_sigma'] = 1
params['y_sigma'] = 1
params['pose_cov'] = 2
n = 10 #num particle
def run(pkl = '../datasets/Jar/dataset1.pkl'):

  
  dataloader = pickle.load(open(pkl,'rb'))
  robotData = dataloader.robots[ROBOT_ID]
  params['initial_pose'][0] = robotData.getXTruth(0)
  params['initial_pose'][1] = robotData.getYTruth(0)
  
  algorithm = FastSLAM2.FastSLAM2(n=n, params=params)
  time_list = []
  all_pose_hist = []
  ground_truth_ls = []
  for i in range(NUM_STEPS):
    update = robotData.getNext()
    
    t = update[1][0]
    #print('step',i)
    if i==0:
      algorithm.prev_t = t
      algorithm.params["initial_pose"][2] = wrapToPi(robotData.getCompass(t))
    if update[0] == "odometry":
      odometry = update[1]
      theta_meas = wrapToPi(robotData.getCompass(t))

      # Use groundtruth to calculate odometry input
      time, velocity, angular_velocity = odometry
      vx = velocity * np.cos(theta_meas)
      vy = velocity * np.sin(theta_meas)
      theta_imu = theta_meas
      omega_imu = angular_velocity

      # Update particle poses
      all_poses = algorithm.propagateMotion((time, vx, vy, theta_imu, omega_imu))

    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement

      # Update EKFs
      if subject > 5:
        # a list of (x,y), where each (x,y) comes from a particle 
        all_poses = algorithm.updateMeasurement((time, range_meas, bearing_meas), MEAS_COV)
    all_pose_hist.append(all_poses)
    for poses in all_poses[:1]:
      plt.plot(poses[0], poses[1],'ro')
    
    plt.plot(robotData.getXTruth(t),robotData.getYTruth(t),'bx')
  plt.plot(robotData.getXTruth(0),robotData.getYTruth(0),'gx')  
  plt.plot(all_pose_hist[0][0][0],all_pose_hist[0][0][1],'go')      
  plt.show()


def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi

  if th <= -np.pi:
      th += 2*np.pi
  return th

run()