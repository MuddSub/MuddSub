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
  p_0: new_land_threshold --> Likelihood of a new feature. When p_0 > p_nt for all p_nt, we have observed a new landmark
  '''
  def __init__(self, particle_id, params, random=None):
    self.id = particle_id
    # self.random = np.random.default_rng(seed)
    self.random = random
    self.next_idx = 0
    self.weight = 0
    self.accumulated_weight = 0

    # self.pose = np.zeros(7)
    self.pose = params['initial_pose']
    
    # self.pose_cov = np.eye(7)
    # self.pose_cov = params['pose_cov']

    self.num_landmarks = params['num_landmarks']
    self.landmarks = {} # id: EFK

    self.new_land_threshold = params['new_land_threshold']

    self.x_sigma = params['x_sigma']
    self.y_sigma = params['y_sigma']
    self.theta_sigma = params['theta_sigma']
    self.v_sigma = params['v_sigma']
    self.omega_sigma = params['omega_sigma']

    self.default_pose_cov = np.diag([self.x_sigma, self.y_sigma, self.theta_sigma, self.v_sigma, self.v_sigma, self.omega_sigma, self.theta_sigma])

    self.observed_land_idx = None
  # need to modify data association to match observation with specific features 
  # possible solution: create a dictionary of landmark class, each has a list of landmark in that class
  # instead of maximizing in a single update, use update if above certain threshold
  # to enable multiple updates. 
  
  def motionUpdate(self, control, dt):
    # Pose estimate that is passed to each landmark EKF which uses it to calculate the sampling distribution and 
    # the probability of data assocation
    #print("Particle: propagate motion:\n control", control, "dt", dt)
    self.pose = self.computeMotionModel(self.pose, control, dt)
    #print("Particle: propagate motion:\n pose", self.pose)
  
  # def addPoseNoise(self):
  #   """Add noise to existing pose"""
  #   noise = self.computePoseNoise()
  #   self.pose = self.pose + noise
  
  def measurementUpdate(self, meas_ls, meas_cov_ls, sensor_range_ls, known_correspondences = False, correspondences = []):
    '''
    For best results, meas_ls should be sorted by range from closest to furthest.
    1. sample paricle pose per landmark 
    2. find the most probable observed landmark if we have unknown correspondences
    3. update particle pose using observed landmark 
    4. update landmark using sampled pose, pose mean, and pose covariance 
    '''
    # Initialize pose mean, pose covariance, and the particle's weight
    self.pose_mean = self.pose
    self.pose_cov = np.copy(self.default_pose_cov)
    self.weight = 1.0

    self.correpondences = correspondences

    # If there are no measurements, sample the pose according to the default pose covariance and the pose mean computed by the motion model
    if len(meas_ls) == 0:
      self.pose = self.pose_mean + self.computePoseNoise()
      return

    # TODO: if a correspondance has low enough class probability, then we should do data association. 

    # Iteratively update pose mean and pose covariance using each measurement of the same time step
    for meas_idx, (meas, meas_cov) in enumerate(zip(meas_ls, meas_cov_ls)):
      # Perform data association
      if not known_correspondences:
        prob_associate_ls = []
        land_idx_ls = [] # keep a separate list to enable mismatch betwee known and unknown correspondences
        for land_idx, landmark in self.landmarks.items():
          prob_associate = landmark.samplePose(self.pose_mean, self.pose_cov, meas, meas_cov)
          prob_associate_ls.append(prob_associate)
          land_idx_ls.append(land_idx)

        # Pick landmark according to maximum likelihood
        ml_associated = 0  
        to_create_new_landmark = False
        if len(prob_associate_ls) > 0:
          ml_idx = np.argmax(prob_associate_ls)
          ml_associated = prob_associate_ls[ml_idx]
          
        # If we dont have any landmark, or assocation probability is really low, then we are seeing new landmarks 
        if len(prob_associate_ls) == 0 or ml_associated < self.new_land_threshold:
          # Initialize new landmark
          observed_land_idx = len(land_idx_ls) + 1
          observed_landmark = LandmarkEKF(random=self.random) 
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.updateNewLandmark(self.pose + self.computePoseNoise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        else:
          # Update observed landmark
          observed_land_idx = land_idx_ls[ml_idx]
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.updateObservedLandmark(self.pose_cov)

          
      else:
        # Use the given correspondence
        observed_land_idx = correspondences[meas_idx]

        # Check if we have the landmark, otherwise create a new one
        if observed_land_idx in self.landmarks:
          # Update observed landmark
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.updateObservedLandmark(self.pose_cov)
        else:
          # Initialize new landmark
          observed_landmark = LandmarkEKF(random=self.random)
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.updateNewLandmark(self.pose + self.computePoseNoise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        ml_idx = observed_land_idx

      # Update unobserved landmarks
      landmark_idxs_to_remove = []
      for landmark_idx, landmark in self.landmarks.items():
        # Don't update observed landmark
        if landmark_idx != observed_land_idx:
          keep = landmark.updateUnobservedLandmark(sensor_range_ls[meas_idx])
          if not keep:
            landmark_idxs_to_remove.append(landmark_idx)

      # Remove unobserved landmarks whose log odds probability of existing fell below 0
      for idx in landmark_idxs_to_remove:
        self.landmarks.pop(idx)
    
      # Update particle properties
      self.weight *= observed_landmark.weight
      self.pose_mean = observed_landmark.pose_mean
      self.pose_cov = observed_landmark.pose_cov
      self.pose = observed_landmark.sampled_pose

  '''
  def updateEKFs(self, meas_ls, meas_cov):
    # TODO Initialize landmarks label/landmark key
    # we might not need this -- we have accurate data association 

    # Get list of data association likelihoods
    prob_associate_ls = []
    land_idx_ls = []
    for land_idx, landmark in self.landmarks.items():
      for meas in meas_ls:
        prob_associate, self.pose, self.pose_cov = landmark.samplePose(self.pose, self.pose_cov, meas, meas_cov)
      prob_associate_ls.append(prob_associate)
      land_idx_ls.append(land_idx)
    prob_associate_ls = np.array(prob_associate_ls)
    # prob_associate_ls = np.array(prob_associate_ls)/sum(prob_associate_ls)

    # Get the index of the landmark with the maximum data association likelihood
    self.observed_land_idx = None
    ml_associate = 0
    if len(prob_associate_ls) > 0:
      ml_idx = np.argmax(prob_associate_ls)
      ml_associate = prob_associate_ls[ml_idx]
      self.observed_land_idx = land_idx_ls[ml_idx]
    if len(prob_associate_ls) > 0:
      print("prob_associate_ls, len", len(prob_associate_ls), 'max', ml_associate)
    
    # List for landmarks the algorithm determines we should remove
    to_remove = []

    # If we have no landmarks or if the observed landmark's data association probability is below a threshold, 
    # create a new landmark. Otherwise, update the observed landmark. Both cases update unobserved landmarks as well.
    if len(prob_associate_ls) == 0 or ml_associate < self.new_land_threshold:
      # Update new landmarks
      print('Add new landmarks')
      for (idx, landmark) in self.landmarks.items():
        keep = landmark.updateUnobserved(self.sensor_range)
        if not keep:
          to_remove.append(idx)

      # Initialize new landmark EKF
      new_landmark = LandmarkEKF(random=self.random)
      noise = self.computePoseNoise()
      #print('noise', noise)
      sampled_pose = self.pose + noise
     
      new_landmark.updateNewLandmark(sampled_pose, meas, meas_cov)
      self.landmarks[self.next_idx]=new_landmark
      self.next_idx+=1

      # Update particle pose and weight
      self.pose = sampled_pose
      self.weight = self.new_land_threshold
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

  def computeMotionModel(self, prev_pose, control, dt):
    vx, vy, theta_imu, omega_imu = control
    prev_x, prev_y, prev_theta, prev_vx, prev_vy, prev_omega, prev_v_p = prev_pose
    
    #v = (vx**2 + vy**2)**.5
    x = prev_x + vx * dt
    y = prev_y + vy * dt
    theta = wrapToPi(theta_imu)
    # vx = v * np.cos(theta)
    # vy = v * np.sin(theta)
    omega = omega_imu #(theta - prev_theta) / dt # NOT SURE
    # so far we are not accounting for omega_imu in jocabian
    theta_p = prev_theta

    pose = np.array([x, y, theta, vx, vy, omega, theta_p])
    
    return pose

  def computePoseNoise(self):
      return self.random.multivariate_normal(np.zeros(7), self.pose_cov)
      