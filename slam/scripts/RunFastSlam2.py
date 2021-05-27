import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import pickle
from Dataloader import *
import FastSLAM2
import numpy as np
import matplotlib.pyplot as plt
ROBOT_ID = 0
START_STEP = 0
NUM_STEPS = 900
MEAS_COV = np.diag([.1, .1])
SENSOR_RANGE = 1
params = {}
params['initial_pose'] = np.array([3.55, -3.38, 0, 0, 0, 0, 0])
params['num_landmarks'] = 0
params['new_land_threshold'] = .05
#TODO Figure out how to get variance for x and y
params['x_sigma'] = 0.001
params['y_sigma'] = 0.001
params['theta_sigma'] = 0.001
params['v_sigma'] = 0.001#0.04
params['omega_sigma'] = 0.001
# params['pose_cov'] = 0.001
n = 10 #num particle
random_generator = np.random.default_rng(0)
plotting = 'best' #vs 'avg'

def runFastSlam2(pkl = '../datasets/Jar/dataset1.pkl'):
  dataloader = pickle.load(open(pkl,'rb'))
  robotData = dataloader.robots[ROBOT_ID]
  params['initial_pose'][0] = robotData.getXTruth(0)
  params['initial_pose'][1] = robotData.getYTruth(0)
  
  algorithm = FastSLAM2.FastSLAM2(n=n, params=params,random=random_generator)
  time_list = []
  all_pose_hist = []
  ground_truth_ls = []
  theta = 0
  for i in range(NUM_STEPS):
    theta_p = theta
    update = robotData.getNext()
    
    t = update[1][0]
    
    if i==0:
      algorithm.prev_t = t
      algorithm.params["initial_pose"][2] = wrapToPi(robotData.getCompass(t))
    if update[0] == "odometry":
      #print("step",i,"updated odometry")
      odometry = update[1]
      theta_meas = wrapToPi(robotData.getCompass(t))

      # Use groundtruth to calculate odometry input
      time, velocity, angular_velocity = odometry
      vx = velocity * np.cos(theta_meas)
      vy = velocity * np.sin(theta_meas)
      theta_imu = theta_meas
      omega_imu = angular_velocity

      # Update particle poses
      algorithm.addControl((vx, vy, theta_imu, omega_imu),t)

      # hard coding poses
      # for i in range(len(algorithm.particles)):
      #   x = robotData.getXTruth(t)
      #   y = robotData.getYTruth(t)
      #   theta = robotData.getCompass(t)
      #   algorithm.particles[i].pose = np.array([x, y, theta, vx, vy, omega_imu, theta_p])
      
    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement

      # Update EKFs
      if subject > 5:
        # a list of (x,y), where each (x,y) comes from a particle 
        algorithm.addMeasurement((range_meas, bearing_meas), MEAS_COV, SENSOR_RANGE)
        print("step", i, "updated measurement")

    all_poses = [ algorithm.particles[i].pose for i in range(len(algorithm.particles))]
    
    # Plot landmark positions
    '''
    landmarks_pos = []
    for particle in algorithm.particles:
      for _, landmark in particle.landmarks.items():
        landmarks_pos.append(np.array(list(landmark.land_mean)))
    landmarks_pos = np.array(landmarks_pos)
    if len(landmarks_pos)>0:
      plt.plot(landmarks_pos[:,0],landmarks_pos[:,1],'yo')
    '''
    # Get average poses
    avg_poses = np.average(all_poses,axis=0)
    all_pose_hist.append(all_poses)
    
    # Plot best particle landmarks
    particle = max(algorithm.particles, key=lambda p: p.accumulated_weight)
    if i == NUM_STEPS - 1:
      #particle = algorithm.particles.index(max(particle.weight for particle in algorithm.particles))
      landmarks_pos = []
      for _, landmark in particle.landmarks.items():
        landmarks_pos.append(np.array(list(landmark.land_mean)))
      landmarks_pos = np.array(landmarks_pos)
      if len(landmarks_pos) > 0:
        plt.plot(landmarks_pos[:,0], landmarks_pos[:,1], 'mo')

    # Plot particle positions based on value of plotting
    to_plot = avg_poses if plotting == 'avg' else particle.pose
    if i > 0.7 * NUM_STEPS and i % 10 == 0:
      plt.plot(to_plot[0], to_plot[1],'ro')
      plt.plot(robotData.getXTruth(t),robotData.getYTruth(t),'bx')
      #plt.annotate(t,(to_plot[0], to_plot[1]))
    elif i % 20 == 0:
      plt.plot(to_plot[0], to_plot[1],'ro')
      plt.plot(robotData.getXTruth(t),robotData.getYTruth(t),'bx')
  
  plt.plot(robotData.getXTruth(0),robotData.getYTruth(0),'gx')
  plt.plot(all_pose_hist[0][0][0],all_pose_hist[0][0][1],'go')
  landmarksGroundtruth = []
  for _, landmark in dataloader.map.landmarkDict.items():
    landmarksGroundtruth.append(np.array([landmark['X'], landmark['Y']]))
  landmarksGroundtruth = np.array(landmarksGroundtruth)
  plt.plot(landmarksGroundtruth[:, 0], landmarksGroundtruth[:, 1], 'cx')
  print("num landmark: ground truth",len(landmarksGroundtruth)," what we got", len(algorithm.particles[0].landmarks))
  plt.title("num step = " + str(NUM_STEPS))
  plt.show()

def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi

  if th <= -np.pi:
      th += 2*np.pi
  return th

runFastSlam2()