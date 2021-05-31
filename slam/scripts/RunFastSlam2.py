import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import pickle
from Dataloader import *
import FastSLAM2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
ROBOT_ID = 0
START_STEP = 0
NUM_STEPS = 13000
MEAS_COV = np.diag([1e-2, 1e-2])
SENSOR_RANGE = 1
LANDMARK_NUM = None #14

plot_data = []

params = {}
params['initial_pose'] = np.array([3.55, -3.38, 0, 0, 0, 0, 0])

params['num_landmarks'] = 0
params['land_default_cov'] = MEAS_COV
params['land_means'] = {14: np.array([.96,.71])}
params['land_covs'] = {}
params['new_land_threshold'] = 1e-2
#TODO Figure out how to get variance for x and y
params['x_sigma'] = 1e-3
params['y_sigma'] = 1e-3
params['theta_sigma'] = 1e-3
params['v_sigma'] = 1e-3#0.04
params['omega_sigma'] = 1e-3
# params['pose_cov'] = 0.001
n = 1 #num particle
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
      #'''
      for i in range(len(algorithm.particles)):
        x = robotData.getXTruth(t)
        y = robotData.getYTruth(t)
        theta = robotData.getCompass(t)
        algorithm.particles[i].pose = np.array([x, y, theta, vx, vy, omega_imu, theta_p])
      #'''
    else:
      measurement = update[1]
      time, subject, range_meas, bearing_meas = measurement

      # Update EKFs
      if (LANDMARK_NUM ==None and subject>5) or subject == LANDMARK_NUM: #if subject > 5 :
        # a list of (x,y), where each (x,y) comes from a particle 
        algorithm.addMeasurement((range_meas, bearing_meas), MEAS_COV, SENSOR_RANGE, subject)
        print("step", i, "updated measurement")

    
    
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

    '''
    # Get average poses
    all_poses = [ algorithm.particles[i].pose for i in range(len(algorithm.particles))]
    avg_poses = np.average(all_poses,axis=0)
    all_pose_hist.append(all_poses)

    # Plot particle positions based on value of plotting
    best_particle = max(algorithm.particles, key=lambda p: p.accumulated_weight)
    to_plot = avg_poses if plotting == 'avg' else best_particle.pose
    if i > 0.7 * NUM_STEPS and i % 10 == 0:
      plt.plot(to_plot[0], to_plot[1],'ro')
      plt.plot(robotData.getXTruth(t),robotData.getYTruth(t),'bx')
      #plt.annotate(t,(to_plot[0], to_plot[1]))
    elif i % 20 == 0:
      plt.plot(to_plot[0], to_plot[1],'ro')
      plt.plot(robotData.getXTruth(t),robotData.getYTruth(t),'bx')
    '''
    
    # Log data
    best_particle = max(algorithm.particles, key=lambda p: p.accumulated_weight)
    particle_poses = []
    landmark_means = []
    landmark_covs = []
    
    for particle_idx, particle in enumerate(algorithm.particles):
      if particle is best_particle:
        best_particle_idx = particle_idx
      particle_poses.append(np.copy(particle.pose))
      
    for _, landmark in best_particle.landmarks.items():
      landmark_means.append(np.copy(landmark.land_mean))
      landmark_covs.append(np.copy(landmark.land_cov))
    frame = (np.array(particle_poses), np.array(landmark_means), np.array(landmark_covs), best_particle_idx, t)
    plot_data.append(frame)

  # Plot best particle landmarks at the end
  #particle = algorithm.particles.index(max(particle.weight for particle in algorithm.particles))
  # plot 
  '''
  best_particle = max(algorithm.particles, key=lambda p: p.accumulated_weight)
  landmarks_pos = []
  for _, landmark in best_particle.landmarks.items():
    landmarks_pos.append(np.array(list(landmark.land_mean)))
  landmarks_pos = np.array(landmarks_pos)
  if len(landmarks_pos) > 0:
    print("plotting landmark")
    plt.plot(landmarks_pos[:,0], landmarks_pos[:,1], 'go')
  else:
    print(landmarks_pos)
  
  plt.plot(robotData.getXTruth(0),robotData.getYTruth(0),'yx')
  plt.plot(all_pose_hist[0][0][0],all_pose_hist[0][0][1],'yo')
  '''

  # Extract landmark groundtruth from dataloader
  landmarksGroundtruth = []
  for idx, landmark in dataloader.map.landmarkDict.items():
    if LANDMARK_NUM == None or idx == LANDMARK_NUM:
      landmarksGroundtruth.append(np.array([landmark['X'], landmark['Y']]))
  landmarksGroundtruth = np.array(landmarksGroundtruth)
  
  # Set up animation
  fig, ax = plt.subplots()
  particles, = ax.plot([], [], marker='o', markerfacecolor='mediumpurple')
  landmarks, = ax.plot([], [], 'mo')
  groundtruth_path_x = []
  groundtruth_path_y = []
  groundtruth_path, = ax.plot([], [], 'bo')
  best_particle_path_x = []
  best_particle_path_y = []
  best_particle_path, = ax.plot([], [], 'ro')
  ax.plot(landmarksGroundtruth[:, 0], landmarksGroundtruth[:, 1], 'cx')
  title = ax.set_title("Step = 0 / " + str(NUM_STEPS))

  def init():
    return particles, landmarks, groundtruth_path, best_particle_path, title

  def update(frame):
    if frame == 0:
      groundtruth_path_x = []
      groundtruth_path_y = []
      best_particle_path_x = []
      best_particle_path_y = []

    particle_poses, landmark_means, landmark_covs, best_particle_idx, t = plot_data[frame]

    # Plot best particle path
    best_particle_pose = particle_poses[best_particle_idx]
    best_particle_path_x.append(best_particle_pose[0])
    best_particle_path_y.append(best_particle_pose[1])
    best_particle_path.set_data(best_particle_path_x, best_particle_path_y)

    # Plot groundtruth path
    groundtruth_path_x.append(robotData.getXTruth(t))
    groundtruth_path_y.append(robotData.getYTruth(t))
    groundtruth_path.set_data(groundtruth_path_x, groundtruth_path_y)
    
    # Plot particles poses
    particles.set_data(particle_poses[:, 0], particle_poses[:, 1])

    # Plot landmarks
    if len(landmark_means) > 0:
      landmarks.set_data(landmark_means[:, 0], landmark_means[:, 1])

    # Update title
    title = ax.set_title("Step = " + str(frame) + " / " + str(NUM_STEPS))

    # Return changed artists?
    return particles, landmarks, groundtruth_path, best_particle_path, title
  
  # plt.plot(landmarksGroundtruth[:, 0], landmarksGroundtruth[:, 1], 'cx')
  print("num landmark: ground truth",len(landmarksGroundtruth)," what we got", len(algorithm.particles[0].landmarks))
  ax.set_title("num step = " + str(NUM_STEPS))
  ani = FuncAnimation(fig, update, frames=range(0, NUM_STEPS, 10), init_func = init, blit=True, interval = 25)
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