import numpy as np
from scipy.linalg import sqrtm
from Landmark import *
def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi
  if th <= -np.pi:
      th += 2*np.pi
  return th
class Particle():
  '''
  params: a dictionary. contains keys:
    initial_pose: initial pose vector
    num_landmarks: initial number of landmarks
    v_sigma: velocity variance
    omega_sigma: angular velocity variance
    theta_sigma: angle variance
  
  Note: use deep copy!
  p_0: prob_new_land --> Likelihood of a new feature. When p_0 > p_nt for all p_nt, we have observed a new landmark
  '''
  def __init__(self, particle_id, params, random=None):
    self.id = particle_id
    # self.random = np.random.default_rng(seed)
    self.random = random
    self.next_idx = 0
    self.weight = 0

    # self.pose = np.zeros(7)
    self.pose = params['initial_pose']
    
    self.pose_cov = np.eye(7)
    # self.pose_cov = params['pose_cov']

    self.num_landmarks = params['num_landmarks']
    self.landmarks = {} # id: EFK

    self.v_sigma = params['v_sigma']
    self.omega_sigma = params['omega_sigma']
    self.theta_sigma = params['theta_sigma']
    # more! 

    self.prob_threshold = params['prob_threshold']
    self.sensor_range = params['sensor_range']

    self.x_sigma = params['x_sigma']
    self.y_sigma = params['y_sigma']

    self.pose_cov = np.diag([self.x_sigma, self.y_sigma, self.theta_sigma, self.v_sigma, self.v_sigma, self.omega_sigma, self.theta_sigma])
    
    self.observed_land_idx = None
  # need to modify data association to match observation with specific features 
  # possible solution: create a dictionary of landmark class, each has a list of landmark in that class
  # instead of maximizing in a single update, use update if above certain threshold
  # to enable multiple updates. 
  
  def propagateMotion(self, control, dt):
    # Pose estimate that is passed to each landmark EKF which uses it to calculate the sampling distribution and 
    # the probability of data assocation
    #print("Particle: propagate motion:\n control", control, "dt", dt)
    self.pose = self.computeMotionModel(self.pose, control, dt)
    #print("Particle: propagate motion:\n pose", self.pose)

  def updateEKFs(self, meas, meas_cov):
    # TODO Initialize landmarks label/landmark key
    # we might not need this -- we have accurate data association 

    # Get list of data association probabilities
    prob_associate_ls = []
    for _,landmark in self.landmarks.items():
      prob_associate = landmark.samplePose(self.pose, self.pose_cov, meas, meas_cov)
      prob_associate_ls.append(prob_associate)
    # prob_associate_ls = np.array(prob_associate_ls)/sum(prob_associate_ls)
    # Get the index of the landmark with the maximum data association probability
    self.observed_land_idx = None
    if len(prob_associate_ls) > 0:
      self.observed_land_idx = np.argmax(np.array(prob_associate_ls))
    print("prob_associate_ls",prob_associate_ls)
    # Store landmarks we want to remove
    to_remove = []

    # If we have no landmarks or if the observed landmark's data association probability is below a threshold, 
    # create a new landmark. Otherwise, update the observed landmark. Both cases update unobserved landmarks as well.
    if len(prob_associate_ls) == 0 or prob_associate_ls[self.observed_land_idx] < self.prob_threshold:
      # Update new landmarks
      print('Update new landmarks')
      for (idx, landmark) in self.landmarks.items():
        keep = landmark.updateUnobserved(self.sensor_range)
        if not keep:
          to_remove.append(idx)

      # Initialize new landmark EKF
      new_landmark = LandmarkEKF(random=self.random)
      noise = self.computePoseNoise()
      print('noise', noise)
      sampled_pose = self.pose + noise
     
      new_landmark.updateNewLandmark(sampled_pose, meas, meas_cov)
      self.landmarks[self.next_idx]=new_landmark
      self.next_idx+=1

      # Update particle pose and weight
      self.pose = sampled_pose
      self.weight = self.prob_threshold
    else:
      print('Update existing landmarks')
      # Loop through landmarks and update the observed one and unobserved ones
      for idx, landmark in self.landmarks.items():
        if idx == self.observed_land_idx:
          self.pose = landmark.sampled_pose
          self.weight = landmark.updateObserved()
        else:
          keep = landmark.updateUnobserved(self.sensor_range)
          if not keep:
            to_remove.append(idx)

    # Remove unobserved landmarks whose log odds probability of existing fell below 0
    for idx in to_remove:
      self.landmarks.pop(idx)
    
    # Return the particle's weight
    return self.weight
  '''
  def correct(self,landmark_key = None, new_landmark=False):
    if new_landmark:
      landmark_key = len(self.landmarks)
      landmark = LandmarkEKF(seed = int(self.random.random()*10000))
    else:
      pass
      # particle correction 
    # find nearby landmark 
    # and downweight 
  '''
  def computeMotionModel(self, prev_pose, control, dt):
    vx, vy, theta_imu, omega_imu = control
    prev_x, prev_y, prev_theta, prev_vx, prev_vy, prev_omega, prev_v_p = prev_pose
    
    v = (vx**2+vy**2)**.5
    x = prev_x + vx * dt
    y = prev_y + vy * dt
    theta = wrapToPi(theta_imu)
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    omega = omega_imu #(theta - prev_theta) / dt # NOT SURE
    # so far we are not accounting for omega_imu in jocabian
    theta_p = prev_theta

    pose = np.array([x,y, theta, vx, vy, omega, theta_p])
    
    return pose

  def computePoseNoise(self):
      return self.random.multivariate_normal(np.zeros(7), self.pose_cov)