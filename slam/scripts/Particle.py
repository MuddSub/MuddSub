import numpy as np
from scipy.linalg import sqrtm
# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)

'''
s: pose --> robot pose (7 states)
sigma_s_t,n_t: pose_cov_expected --> the pose cov for the **new** pose sampling distribution in data association.  
sigma_s_t,n_t: pose_mean_expected --> the pose mean for the **new** pose sampling distribution in data association. 
s_nt,t: sampled pose --> the pose sampled from the **new** distribution in data association

mu: land_mean --> landmark position mean (x,y) 
sigma: land_cov --> landmark position covariance

mu_t-1: prev_land_mean --> landmark position mean from previous time step
sigma_t-1: prev_land_cov --> landmark position covariance from previous time step

z: meas = (meas_range, meas_bearing) --> measurement
z_hat: meas_est = (meas_range_est, meas_bearing_est) --> predicted measurement
z - z_hat:  meas_diff

Gs: meas_jac_pose --> Jacobian of the measurement model with respect to the robot pose
Gtheta: meas_jac_land --> Jacobian of the measurement model with respect to the landmark position

g: measurement_model
R: meas_cov: measurement covariance

h: motion_model (on pose)
P: pose_cov: pose (motion model) covariance

tau: land_exist_log --> log odds of the probability that the landmark exists
rho+: land_exist_log_inc --> log odds value to add to land_exist_log when positive evidence of the landmark is seen
rho-: land_exist_log_neg --> log odds value to subtract to land_exist_log when negative evidence of the landmark is seen

Q: Q --> fused measurement covariance. meas_cov + meas_jac_land @ land_cov @ meas_jac_land
    (2,2)
Q: Q_inv --> Q inverse
K: K --> Kalman gain (2,2)
I: identity matrix with the shape of KGtheta, !!!!!! Problem: size mismatched

w: weight 

p_nt: prob_match --> probability of landmark associates with given measurement
'''

class LandmarkEKF():
  def __init__(self, land_mean=np.zeros(2), land_cov = np.eye(2),seed=0):
    self.prev_land_mean = land_mean
    self.prev_land_cov = land_cov

    self.land_mean = np.zeros(2)
    self.land_cov = np.zeros((2,2))

    # In FastSLAM 2.0, the pose is sampled taking into account the measurement and the data association variable. 
    # If the data association variable is not known, the sampled pose is used to calculate the probability of having
    # observed the landmark given the measurement, and the landmark that maximizes this probability is chosen. That
    # is why the sampled pose is saved in the EKF.
    self.sampled_pose = np.zeros(7)

    self.meas = np.zeros(2)
    self.meas_diff = np.zeros(2)

    self.meas_jac_pose, self.meas_jac_land = np.zeros((2,7)),np.zeros((2,7))

    self.pose_cov, self.meas_cov = np.zeros((7,7)), np.zeros((2,2))
    self.pose_cov_inv = self.pose_cov

    self.Q = np.zeros((2,2))
    self.Q_inv = np.zeros((2,2)) 
    #self.K = np.zeros((2,2))
    self.I = np.eye(2) #!!!!!! Problem: size mismatched

    self.random = np.random.default_rng(seed)
    self.prob_data_association = 0
    
    self.land_exist_log = 0
    self.land_exist_log_threshold = self.logOdds(0.5)
    self.land_exist_log_inc = self.logOdds(0.8) # 0.8 works out to around 1.38, rho positive in the FastSLAM 2.0 algorithm
    self.land_exist_log_dec = self.logOdds(0.2) # 0.2 works out to around -1.38, rho negative in the FastSLAM 2.0 algorithm


  def logOdds(self, probability):
    return np.log(probability / (1 - probability))

  def computeMeasModel(self, pose):
    x,y,theta, vx, vy, omega, theta_p = pose
    
    lx, ly = self.prev_land_mean

    range_est = ((lx-x)**2+(ly-y)**2)**.5
    bearing_est = np.arctan2((ly-y)/(lx-x))-theta

    return np.array([range_est, bearing_est])

  def computeMeasModelInverse(self, pose):
    x, y, theta, vx, vy, omega, theta_p = pose
    range_meas, bearing_meas = self.meas
    lx = x + range_meas * np.cos(theta + bearing_meas)
    ly = y + range_meas * np.sin(theta + bearing_meas)
    return np.array([lx, ly])

  def computeMeasJacobians(self, pose):
    lx, ly = self.prev_land_mean
    x, y, theta, vx, vy, omega, theta_p = pose
    
    range_est_sqr = ((lx-x)**2+(ly-y)**2)
    range_est = range_est_sqr**.5

    meas_jac_pose = np.zeros((2,7))

    meas_jac_pose[0,0] = -(lx-x) / range_est
    meas_jac_pose[1,0] = -(ly-y) / range_est
    meas_jac_pose[0,1] = (ly-y) / range_est_sqr
    meas_jac_pose[1,1] = -(lx-x) / range_est_sqr
    meas_jac_pose[1,2] = -1
    self.meas_jac_pose = meas_jac_pose
    self.meas_jac_land = -1 * meas_jac_pose[:2,:2]
    
  def samplePose(self, pose_est, pose_cov, meas, meas_cov):
    # range_meas, bearing_meas = meas 
    self.meas = meas
    self.meas_cov = meas_cov
    meas_est = self.computeMeasModel(pose_est) # range_est, bearing_est
    self.meas_diff = meas - meas_est #np.array([range_meas - range_est, bearing_meas - bearing_est])

    self.pose_cov = pose_cov
    self.pose_cov_inv = np.linalg.inv(self.pose_cov)

    self.computeMeasJacobians(pose_est)
    
    self.Q = self.meas_cov + self.meas_jac_land @ self.prev_land_cov @ self.meas_jac_land.T
    self.Q_inv = np.linalg.inv(self.Q)
    
    pose_cov_expected = np.linalg.inv(self.meas_jac_pose.T @ self.Q_inv @ self.meas_jac_pose + self.pose_cov_inv)
    pose_mean_expected = pose_cov_expected @ self.meas_jac_pose.T @ self.Q_inv @ self.meas_diff + pose_est

    self.sampled_pose = pose_mean_expected + self.random.normal(0,pose_cov_expected)
    meas_improved = self.computeMeasModel(self.sampled_pose) #range_improved, bearing_improved
    improved_diff = meas - meas_improved 
    exponent = -.5*(improved_diff).T @ self.Q_inv @ (improved_diff)
    
    two_pi_Q_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*self.Q)))
    
    self.prob_data_association = two_pi_Q_inv_sqrt * np.exp(exponent)
    return self.prob_data_association

  def updateObserved(self):
    self.land_exist_log += self.land_exist_log_inc
    K =  self.prev_land_cov @ self.meas_jac_land.T @ self.Q_inv 

    # Compute new landmark mean and covariance
    self.land_mean = self.prev_land_mean + K @ self.meas_diff
    self.land_cov = (self.I - K @ self.meas_jac_land) @ self.prev_land_cov #!!!!!! Problem: size mismatched
    
    # Compute particle weight
    L = self.meas_jac_pose @ self.pose_cov @ self.meas_jac_pose.T \
         + self.meas_jac_land @ self.prev_land_cov @ self.meas_jac_land.T \
         + self.meas_cov
    L_inv = np.linalg.inv(L)
    two_pi_L_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*L)))
    exponent = -.5* self.meas_diff.T @ L_inv @ (self.meas_diff )
    weight = two_pi_L_inv_sqrt * np.exp(exponent)

    # Update previous values
    self.prev_land_mean = self.land_mean
    self.prev_land_cov = self.land_cov

    return weight

  def updateUnobserved(self, sensor_range):
    # Set current landmark mean and cov to previous mean and cov
    self.land_mean = self.prev_land_mean
    self.land_cov = self.prev_land_cov

    # Update probability of the landmark existing based on whether it should have been measured
    range_meas, _ = self.computeMeasModel(self.sampled_pose)
    if range_meas <= sensor_range:
      self.land_exist_log -= self.land_exist_log_dec

    # If the log odds probability falls below 0, we do not keep the landmark
    return self.land_exist_log > 0

  def updateNewLandmark(self, sampled_pose, meas, meas_cov):
    self.land_exist_log = self.land_exist_log_inc
    self.sampled_pose = sampled_pose
    self.meas = meas

    # Initialize landmark mean and covariance, as well as previous mean and covariance
    self.land_mean = self.computeMeasModelInverse(sampled_pose)
    self.prev_land_mean = self.land_mean
    self.computeMeasJacobians(sampled_pose)
    self.land_cov = np.linalg.inv(self.meas_jac_land @ np.linalg.inv(meas_cov) @ self.meas_jac_land.T)
    self.prev_land_cov = self.land_cov

# class State():
#   def __init__(self, x, y, theta):
#     self.x = x
#     self.y = y
#     self.theta = theta

#     self.map = dict()

class Particle():
  '''
  Note: use deep copy!

  p_0: prob_new_land --> Likelihood of a new feature. When p_0 > p_nt for all p_nt, we have observed a new landmark
  '''
  def __init__(self, particle_id, params, seed=0):
    self.id = particle_id
    self.random = np.random.default_rng(seed)
    self.next_idx = 0
    
    self.weight = 0

    self.pose = np.zeros(7)
    self.pose_hist = []
    self.pose_cov = np.eye(7)

    self.num_landmarks = params['num_landmarks']
    self.landmarks = {} # id: EFK

    self.v_sigma = params['v_sigma']
    self.omega_sigma = params['omega_sigma']
    self.theta_sigma = params['theta_sigma']
    # more! 

    self.prob_threshold = .5
    self.sensor_range = 10

    #TODO Figure out how to get variance for x and y
    self.x_sigma = 1
    self.y_sigma = 1

    self.pose_cov = np.diag([self.x_sigma, self.y_sigma, self.theta_sigma, self.v_sigma, self.v_sigma, self.omega_sigma, self.theta_sigma])
    
    pass

  # need to modify data association to match observation with specific features 
  # possible solution: create a dictionary of landmark class, each has a list of landmark in that class
  # instead of maximizing in a single update, use update if above certain threshold
  # to enable multiple updates. 
  def propagateMotion(self, control, meas, meas_cov, landmark_key, dt):
    # Store old pose
    self.pose_hist.append(self.pose)

    # Pose estimate that is passed to each landmark EKF which uses it to calculate the sampling distribution and 
    # the probability of data assocation
    pose_est = self.computeMotionModel(self.pose, control, dt)

    # TODO Initialize landmarks 
    # we might not need this -- we have accurate data association 

    # Get list of data association probabilities
    prob_associate_ls = []
    for _,landmark in self.landmarks.items():
      prob_associate = landmark.samplePose(pose_est, self.pose_cov, meas, meas_cov)
      prob_associate_ls.append(prob_associate)
    
    # Get the index of the landmark with the maximum data association probability
    observed_land_idx = None
    if len(prob_associate_ls) > 0:
      observed_land_idx = np.argmax(np.array(prob_associate_ls))
    
    # Store landmarks we want to remove
    to_remove = []

    # If we have no landmarks or if the observed landmark's data association probability is below a threshold, 
    # create a new landmark. Otherwise, update the observed landmark. Both cases update unobserved landmarks as well.
    if len(prob_associate_ls) == 0 or prob_associate_ls[observed_land_idx] < self.prob_threshold:
      # Update unobserved landmarks
      for (idx, landmark) in self.landmarks.items():
        keep = landmark.updateUnobserved()
        if not keep:
          to_remove.append(idx)

      # Initialize new landmark EKF
      new_landmark = LandmarkEKF()
      sampled_pose = pose_est + self.computePoseNoise()
      new_landmark.updateNewLandmark(sampled_pose, meas, meas_cov)
      self.landmarks[self.next_idx]=new_landmark
      self.next_idx+=1

      # Update particle pose and weight
      self.pose = sampled_pose
      self.weight = self.prob_threshold
    else:
      # Loop through landmarks and update the observed one and unobserved ones
      for idx, landmark in self.landmarks.items():
        if idx == observed_land_idx:
          self.pose = landmark.sampled_pose
          self.weight = landmark.updateObserved()
        else:
          keep = landmark.updateUnobserved()
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
      landmark = LandmarkEKF()
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
    theta = theta_imu
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    omega = (theta - prev_theta) / dt # NOT SURE
    theta_p = prev_theta

    pose = np.array([x,y,theta, vx, vy, omega, theta_p])

    return pose

  def computePoseNoise(self):
      return self.random.normal(0, self.pose_cov, 7)