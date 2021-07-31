import numpy as np
from scipy.linalg import sqrtm
from Landmark import *

def wrap_to_pi(th):
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
  p_0: new_land_threshold --> Likelihood of a new feature. When p_0 > p_nt for all p_nt, we have observed a new landmark
  '''
  def __init__(self, particle_id, params, random=None):
    self.id = particle_id
    # self.random = np.random.default_rng(seed)
    self.random = random
    self.next_idx = 0
    self.weight = 0
    self.accumulated_weight = 0

    self.x_sigma = params['x_sigma']
    self.y_sigma = params['y_sigma']
    self.theta_sigma = params['theta_sigma']
    self.can_change_landmark = params['can_change_landmark']

    self.default_pose_cov = np.diag([self.x_sigma, self.y_sigma, self.theta_sigma])
    self.pose_cov = np.copy(self.default_pose_cov)
    self.pose = params['initial_pose']
    self.add_pose_noise()

    self.num_landmarks = params['num_landmarks']
    self.landmarks = {} # id: EFK

    if self.num_landmarks > 0 and params['land_means'] != None and len(params['land_means'])>0:
      for idx, land_mean in params['land_means'].items():
        land_cov = params['land_covs'].get(idx, params['land_default_cov'])
        self.landmarks[idx] = LandmarkEKF(land_mean=land_mean, land_cov=land_cov, random=self.random)
    
    self.new_land_threshold = params['new_land_threshold']

    self.observed_land_idx = None
  # need to modify data association to match observation with specific features 
  # possible solution: create a dictionary of landmark class, each has a list of landmark in that class
  # instead of maximizing in a single update, use update if above certain threshold
  # to enable multiple updates. 

  
  def add_pose_noise(self):
    """Add noise to existing pose"""
    noise = self.compute_pose_noise()
    self.pose = self.pose + noise

  def compute_pose_noise(self):
    return self.random.multivariate_normal(np.zeros(3), self.pose_cov)
  
  def measurement_update(self, meas_ls, meas_cov_ls, sensor_range_ls, sensor_bearing_ls, known_correspondences = False, correspondences = []):
    '''
    For best results, meas_ls should be sorted by range from closest to furthest.
    1. sample paricle pose per landmark 
    2. find the most probable observed landmark if we have unknown correspondences
    3. update particle pose using observed landmark 
    4. update landmark using sampled pose, pose mean, and pose covariance 
    '''
    # Initialize pose mean, pose covariance, and the particle's weight

    # TO DO: 
    #  remove self? 

    self.pose_mean = np.copy(self.pose)
    self.pose_cov = np.copy(self.default_pose_cov)
    self.weight = 1.0

    self.correpondences = correspondences

    # If there are no measurements, sample the pose according to the default pose covariance and the pose mean computed by the motion model
    if len(meas_ls) == 0:
      noise = self.compute_pose_noise()
      self.pose = self.pose_mean + noise
      return

    # TODO: if a correspondence has low enough class probability, then we should do data association.

    # Iteratively update pose mean and pose covariance using each measurement of the same time step
    for meas_idx, (meas, meas_cov) in enumerate(zip(meas_ls, meas_cov_ls)):
      # Perform data association
      if not known_correspondences:
        prob_associate_ls = []
        land_idx_ls = [] # keep a separate list to enable mismatch betwee known and unknown correspondences
        for land_idx, landmark in self.landmarks.items():
          prob_associate = landmark.sample_pose(self.pose_mean, self.pose_cov, meas, meas_cov)
          prob_associate_ls.append(prob_associate)
          land_idx_ls.append(land_idx)

        # Pick landmark according to maximum likelihood
        ml_associated = 0  
        to_create_new_landmark = False
        if len(prob_associate_ls) > 0:
          ml_idx = np.argmax(prob_associate_ls)
          ml_associated = prob_associate_ls[ml_idx]
          
        # If we dont have any landmark, or assocation probability is really low, then we are seeing new landmarks 
        if len(prob_associate_ls) == 0 or (ml_associated < self.new_land_threshold and self.can_change_landmark):
          # Initialize new landmark
         
          observed_land_idx = len(land_idx_ls) + 1
          observed_landmark = LandmarkEKF(random=self.random) 
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.update_new_landmark(self.pose + self.compute_pose_noise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        else:
          # Update observed landmark
          observed_land_idx = land_idx_ls[ml_idx]
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.update_observed_landmark(self.pose_cov)

      else:
        # Use the given correspondence
        observed_land_idx = correspondences[meas_idx]

        # Check if we have the landmark, otherwise create a new one
        if observed_land_idx in self.landmarks:
          # Update observed landmark
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.sample_pose(self.pose_mean, self.pose_cov, meas, meas_cov)
          observed_landmark.update_observed_landmark(self.pose_cov)
        else:
          # Initialize new landmark
          observed_landmark = LandmarkEKF(random=self.random)
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.update_new_landmark(self.pose + self.compute_pose_noise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        ml_idx = observed_land_idx
      if self.can_change_landmark:
        # Update unobserved landmarks
        landmark_idxs_to_remove = []
        for landmark_idx, landmark in self.landmarks.items():
          # Don't update observed landmark
          if landmark_idx != observed_land_idx:
            keep = landmark.update_unobserved_landmark(sensor_range_ls[meas_idx],sensor_bearing_ls[meas_idx])
            if not keep:
              landmark_idxs_to_remove.append(landmark_idx)

        # Remove unobserved landmarks whose log odds probability of existing fell below 0
        for idx in landmark_idxs_to_remove:
          self.landmarks.pop(idx)
    
      # Update particle properties
      self.weight *= observed_landmark.weight
      self.weight = max(self.weight, 1e-50)
      self.pose_mean = np.copy(observed_landmark.pose_mean)
      self.pose_cov = np.copy(observed_landmark.pose_cov)
      self.pose = np.copy(observed_landmark.sampled_pose)

  def measurement_update_localization(self, meas_ls, meas_cov_ls, sensor_range_ls,sensor_bearing_ls, known_correspondences=False, correspondences=[]):
    self.weight = 1

    # If there are no measurements, sample the pose according to the default pose covariance and the pose mean computed by the motion model
    if len(meas_ls) == 0:
      self.add_pose_noise()
      return
    
    for meas, meas_cov, idx in zip(meas_ls, meas_cov_ls, correspondences):
      observed_landmark = self.landmarks[idx]
      # print("Observed landmark:", idx, "with mean", observed_landmark.land_mean)
      self.weight *= observed_landmark.compute_weight_localization(self.pose, meas, meas_cov)

    # print("Weight is", self.weight)
    self.weight = max(self.weight, 1e-50)

  def motion_update(self, control, dt):
    # Pose estimate that is passed to each landmark EKF which uses it to calculate the sampling distribution and 
    # the probability of data assocation
    self.pose = self.compute_motion_model(self.pose, control, dt)

  def compute_motion_model(self, prev_pose, control, dt):
    v, w = control
    if -1e-10 <= w <= 1e-10:
      w = 1e-10

    x, y, theta = prev_pose
    next_theta = wrap_to_pi(theta + w * dt)
    next_x = x + v/w * (-np.sin(theta) + np.sin(next_theta))
    next_y = y + v/w * ( np.cos(theta) - np.cos(next_theta))
    return np.array([next_x, next_y, next_theta])