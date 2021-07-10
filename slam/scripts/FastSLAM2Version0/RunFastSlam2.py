import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import pickle
from Dataloader import *
import FastSLAM2
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import argparse
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
INIT_LANDMARKS = False
KNOWN_CORRESPONDENCES = True

'''
threshold: .3, meas cov: .1, motion cov: 1e-2
'''

plot_data = []

params = {}
params['initial_pose'] = np.array([3.55, -3.38, 0])

params['num_landmarks'] = 0
params['land_default_cov'] = MEAS_COV
params['land_means'] = {}#{14: np.array([.96,.71])}
params['land_covs'] = {}
# too many particles: lower threshold
params['new_land_threshold'] = .1
#TODO Figure out how to get variance for x and y
params['x_sigma'] = 1e-4
params['y_sigma'] = 1e-4
params['theta_sigma'] = 1e-4
params['can_change_landmark'] = True
n = 50 #num particle
random_generator = np.random.default_rng()
algorithm = None

def runFastSlam2(pkl = '../../datasets/Jar/dataset1.pkl'):
  global algorithm
  dataloader = pickle.load(open(pkl,'rb'))
  robotData = dataloader.robots[ROBOT_ID]
  params['initial_pose'][0] = robotData.getXTruth(0)
  params['initial_pose'][1] = robotData.getYTruth(0)
  params['initial_pose'][2] = robotData.getCompass(0)

  if INIT_LANDMARKS:
    # Load in landmarks
    params['num_landmarks'] = 15
    params['land_means'] = {}
    for idx, landmark in dataloader.map.landmarkDict.items():
      x = landmark['X']
      y = landmark['Y']
      params['land_means'][idx] = np.array([x, y])

  # Create instance of FastSLAM2
  algorithm = FastSLAM2.FastSLAM2(n=n, params=params, random=random_generator)

  # Start loop
  theta = 0
  for i in range(NUM_STEPS):
    
    theta_p = theta
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
      # print("step", i, "updated odometry", (vx, vy, theta_imu, omega_imu))

      # Update particle poses
      # algorithm.addControl((vx, vy, theta_imu, omega_imu), t)
      algorithm.addControl((velocity, angular_velocity), t)

      # Hard coding poses
      if HARDCODE_COMPASS:
        for i in range(len(algorithm.particles)):
          x = algorithm.particles[i].pose[0]
          y = algorithm.particles[i].pose[1]
          theta = robotData.getCompass(t)
          algorithm.particles[i].pose = np.array([x, y, theta])
    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement
      if not NO_MEASUREMENTS:
        # Update EKFs
        if (LANDMARK_NUM == None and subject > 5) or subject == LANDMARK_NUM: #if subject > 5 :
          landmark = dataloader.map.getLandmarkLocation(subject)
          landmark_x = landmark['X']
          landmark_y = landmark['Y']

          # Use groundtruth to provide accurate measurement
          if HARDCODE_MEAS:
            robot_x = robotData.getXTruth(time)
            robot_y = robotData.getYTruth(time)
            robot_angle = robotData.getCompass(time)
            range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
            bearing_meas = wrapToPi(np.arctan2(landmark_y - robot_y, landmark_x - robot_x) - robot_angle)
          print("step", i, "updated measurement", subject, "with position", [landmark_x, landmark_y])

          if KNOWN_CORRESPONDENCES:
            algorithm.addMeasurement((range_meas, bearing_meas), MEAS_COV, SENSOR_RANGE, SENSOR_BEARING, subject)
          else:
            algorithm.addMeasurement((range_meas, bearing_meas), MEAS_COV, SENSOR_RANGE, SENSOR_BEARING)
    
    # Log data
    best_particle = max(algorithm.particles, key=lambda p: p.accumulated_weight)
    particle_poses = []
    landmark_means = []
    landmark_covs = []
    landmark_idxs = []
    
    for particle_idx, particle in enumerate(algorithm.particles):
      if particle is best_particle:
        best_particle_idx = particle_idx
      particle_poses.append(np.copy(particle.pose))

    for idx, landmark in best_particle.landmarks.items():
      landmark_means.append(np.copy(landmark.land_mean))
      landmark_covs.append(np.copy(landmark.land_cov))
      landmark_idxs.append(idx)

    frame = (np.array(particle_poses), np.array(landmark_means), np.array(landmark_covs), best_particle_idx, landmark_idxs, t)
    plot_data.append(frame)

  # Extract landmark groundtruth from dataloader
  landmarksGroundtruth = []
  for idx, landmark in dataloader.map.landmarkDict.items():
    if LANDMARK_NUM == None or idx == LANDMARK_NUM:
      landmarksGroundtruth.append(np.array([landmark['X'], landmark['Y']]))
  landmarksGroundtruth = np.array(landmarksGroundtruth)
  
  print("num landmark: ground truth", len(landmarksGroundtruth), " what we got", len(algorithm.particles[0].landmarks))

  # Set up animation
  fig = plt.figure()
  ax = fig.add_subplot(111)  
  ax.plot(landmarksGroundtruth[:, 0], landmarksGroundtruth[:, 1], 'cx',label='true landmark')
  particles, = ax.plot([], [], linestyle='None', marker='o', color='gold',label='est motion')
  best_particle_landmarks, = ax.plot([], [], 'mo', label='est landmark')
  groundtruth_path_x = []
  groundtruth_path_y = []
  groundtruth_path, = ax.plot([], [], 'b-',label='true path')
  best_particle_path_x = []
  best_particle_path_y = []
  best_particle_path, = ax.plot([], [], 'r-',label='est path')
  steps = ax.text(3, 6, "Step = 0 / " + str(NUM_STEPS), horizontalalignment="center", verticalalignment="top")
  ax.legend()
  def init():
    ax.set_title("Num steps: " + str(NUM_STEPS) + ", Num particle: "+ str(n))
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps

  def update(frame):
    particle_poses, landmark_means, landmark_covs, best_particle_idx, landmark_idxs, t = plot_data[frame]

    if frame == 0:
      groundtruth_path_x.clear()
      groundtruth_path_y.clear()
      best_particle_path_x.clear()
      best_particle_path_y.clear()

    # Plot best particle path
    if PLOT_AVG:
      avg_particle_pose = np.average(particle_poses, axis=0)
      best_particle_path_x.append(avg_particle_pose[0])
      best_particle_path_y.append(avg_particle_pose[1])
    else:
      best_particle_pose = particle_poses[best_particle_idx]
      best_particle_path_x.append(best_particle_pose[0])
      best_particle_path_y.append(best_particle_pose[1])
    best_particle_path.set_data(best_particle_path_x, best_particle_path_y)

    # Plot groundtruth path
    groundtruth_path_x.append(robotData.getXTruth(t))
    groundtruth_path_y.append(robotData.getYTruth(t))
    groundtruth_path.set_data(groundtruth_path_x, groundtruth_path_y)
    
    # Plot other particles poses
    particles.set_data(particle_poses[:, 0], particle_poses[:, 1])

    # Plot best particle landmarks
    if len(landmark_means) > 0:
      best_particle_landmarks.set_data(landmark_means[:, 0], landmark_means[:, 1])

    # Update title
    if frame/NUM_STEPS > .9:
      steps.set_text('')
    else:
      steps.set_text("Step = " + str(frame) + " / " + str(NUM_STEPS))

    # Return changed artists?
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps
  
  anim = animation.FuncAnimation(fig, update, frames=range(0, NUM_STEPS, 20), init_func = init, blit=False, interval = 33, repeat=False)
  fig.tight_layout(rect=[0, 0.03, 1, 0.95])
  plt.show()
  print('Saving animation...')
  f = r'../animations/FastSLAM2_3.gif'
  writergif = animation.PillowWriter(fps=30)
  anim.save(f, writer=writergif)
  print('Animation Saved!')
  if KNOWN_CORRESPONDENCES:
    
    # sqrted differences between the (pose-landmark distance) 
    # for ground truth and estimation 
    sqrt_pose_landmark_dist_error = [] 

    for particle_poses, landmark_means, landmark_covs, best_particle_idx, landmark_idxs, time in plot_data:
          #print("time step",time, "landmark means length",len(landmark_means))
          #print(landmark_idxs,landmark_means)
          if len(landmark_means)==0:
            continue
          # ground truth data
          ground_truth_pose_landmark_dist = []
          estimated_pose_landmark_dist = []
          robot_x = robotData.getXTruth(time)
          robot_y = robotData.getYTruth(time)
          robot_angle = robotData.getCompass(time)  
          for idx, (landmark_x, landmark_y) in enumerate(landmarksGroundtruth):
            if idx+6 in landmark_idxs:
              range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
              ground_truth_pose_landmark_dist.append(range_meas)
          # fast slam data
          best_particle_pose = particle_poses[best_particle_idx]
          best_particle_x = best_particle_pose[0]
          best_particle_y = best_particle_pose[1]      
          for landmark_x, landmark_y in landmark_means:
            range_meas = ((best_particle_x - landmark_x) ** 2 + (best_particle_y - landmark_y) ** 2) ** 0.5
            estimated_pose_landmark_dist.append(range_meas)      
          ground_truth_pose_landmark_dist = np.array(ground_truth_pose_landmark_dist)
          estimated_pose_landmark_dist = np.array(estimated_pose_landmark_dist)
          sqrt_pose_landmark_dist_error.append(np.average((ground_truth_pose_landmark_dist-estimated_pose_landmark_dist)**2))
    sqrt_pose_landmark_dist_error = np.array(sqrt_pose_landmark_dist_error)
    print("sqrt_pose_landmark_dist_error",sum(sqrt_pose_landmark_dist_error**.5))
    print("avg",np.mean(sqrt_pose_landmark_dist_error**.5))
    print("median",np.median(sqrt_pose_landmark_dist_error**.5))
    print("final",sqrt_pose_landmark_dist_error[-1]**.5)
    plt.plot(sqrt_pose_landmark_dist_error**.5)
    plt.show()


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

def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi

  if th <= -np.pi:
      th += 2*np.pi
  return th

runFastSlam2()