import pickle
from Dataloader import *
import FastSLAM2
import numpy as np
import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
ROBOT_ID = 0
NUM_STEPS = 1
MEAS_COV = np.diag([0.075, 0.025])

n = 10 #num particle
def run(pkl = '../datasets/Jar/dataset1.pkl'):

  algorithm = FastSLAM2.FastSLAM2(n=n)
  dataloader = pickle.load(open(pkl,'rb'))
  robotData = dataloader.robots[ROBOT_ID]
  
  time_list = []
  for i in range(NUM_STEPS):
    update = robotData.getNext()
    t = update[1][0]
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
      algorithm.propagateMotion((time, vx, vy, theta_imu, omega_imu))
    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement

      # Update EKFs
      if subject > 5:
        algorithm.updateMeasurement((time, range_meas, bearing_meas), MEAS_COV)


def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi

  if th <= -np.pi:
      th += 2*np.pi
  return th

run()