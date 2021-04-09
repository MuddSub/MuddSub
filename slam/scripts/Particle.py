import numpy as np
from scipy.linalg import sqrtm
# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)


'''
s: pose --> robot pose (7 states)
sigma_s_t,n_t: pose_cov_expected: the pose cov for the **new** distribution in data association.  
sigma_s_t,n_t: pose_mean_expected: the pose mean for the **new** distribution in data association. 
s_nt, t: sampled pose: the pose sampled from the **new** distribution from data association 

mu: land_mean --> landmark position mean (x,y) 
sigma: land_cov --> landmark position covariance

mu_hat: land_mean_est --> landmark position mean estimate (prediction, before correction step)
sigma_hat: land_cov_est --> landmark position covariance estimate (prediction, before correction step)

z: meas = (meas_range, meas_bearing) --> measurement
z_hat: meas_est = (meas_range_est, meas_bearing_est) --> predicted measurement
z-z_hat:  meas_diff 

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

    self.land_mean_est = np.zeros(2)
    self.land_cov_est = np.zeros((2,2))

    self.sampled_pose = np.zeros(7)

    self.meas = np.zeros(2)
    self.meas_diff = np.zeros(2)

    self.meas_jac_pose, self.meas_jac_land = np.zeros((2,7)),np.zeros((2,7))

    self.pose_cov, self.meas_cov = np.zeros((7,7)), np.zeros((2,2))
    self.pose_cov_inv = self.pose_cov

    self.Q = np.zeros((2,2))
    self.Q_inv = np.zeros((2,2)) 
    #self.K = np.zeros((2,2))
    self.I = np.zeros((2,2)) #!!!!!! Problem: size mismatched

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
    
  def computeDataAssociation(self, pose_est, pose_cov, meas, meas_cov):

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
    meas_improved = self.computeMeasModel(pose_sampled) #range_improved, bearing_improved
    improved_diff = meas - meas_improved 
    exponent = -.5*(improved_diff).T @ self.Q_inv @ (improved_diff)
    
    two_pi_Q_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*self.Q)))
    
    self.prob_data_association = two_pi_Q_inv_sqrt * np.exp(exponent)
    return self.prob_data_association

  def updateObserved(self):

    self.land_exist_log += self.land_exist_log_inc

    K =  self.prev_land_cov @ self.meas_jac_land.T @ self.Q_inv 

    self.land_mean_est = self.prev_land_mean + K @ self.meas_diff

    self.land_cov_est = (self.I - K @ self.meas_jac_land) @ self.prev_land_cov #!!!!!! Problem: size mismatched

    L = self.meas_jac_pose @ self.pose_cov @ self.meas_jac_pose.T \
         + self.meas_jac_land @ self.prev_land_cov @ self.meas_jac_land.T \
         + self.meas_cov
    L_inv = np.linalg.inv(L)
    two_pi_L_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*L)))
    exponent = -.5* self.meas_diff.T @ L_inv @ (self.meas_diff )
    weight = two_pi_L_inv_sqrt * np.exp(exponent)

    return weight

  def updateUnobserved(self,pose,sensor_range):
    range_meas, _ = self.computeMeasModel(pose)
    if range_meas <= sensor_range:
      self.land_exist_log -= self.land_exist_log_dec
    if self.land_exist_log<=0:
      return False
    return True

  def updateNewLandmark(self, pose_est, pose_cov_est, meas, meas_cov):
    pass

class State():
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta

    self.map = dict()  

class Particle():
  '''
  Note: use deep copy!

  p_0: prob_new_land --> Likelihood of a new feature. When p_0 > p_nt for all p_nt, we have observed a new landmark
  '''
  def __init__(self, state,params, particle_id, seed=0):
    self.id = particle_id
    self.random = np.random.default_rng(seed)
    self.next_idx = 0
    
    self.weight = 0

    self.pose = np.zeros(7)
    self.pose_hist = []
    self.pose_cov = np.eyes(7)

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

    self.pose_cov = np.eyes([self.x_sigma, self.y_sigma, self.theta_sigma, self.v_sigma, self.v_sigma, self.omega_sigma, self.theta_sigma])
    
    pass

  # need to modify data association to match observation with specific features 
  # possible solution: create a dictionary of landmark class, each has a list of landmark in that class
  # instead of maximizing in a single update, use update if above certain threadhold 
  # to enable multiple updates. 
  def propagateMotion(self, control, meas, meas_cov, landmark_key, dt):
    self.pose_hist.append(self.pose)
    pose_est = self.computeMotionModel(self.pose, control, dt)
    # TO DO INItiALIze landmarks 
    # we might not need this -- we have accurate data association 
    prob_associate_ls = []
    for _,landmark in self.landmarks.items():
      prob_associate = landmark.computeDataAssociation(pose_est, self.pose_cov, meas, meas_cov)
      prob_associate_ls.append(prob_associate)
    
    observed_land_idx = None
    if len(prob_associate_ls)>0:
      observed_land_idx = np.argmax(np.array(prob_associate_ls))
    
    if len(prob_associate_ls)==0 or prob_associate_ls[observed_land_idx] < self.prob_threshold:
      for landmark in self.landmarks:
        landmark.updateUnobserved(pose_est)
      self.weight = self.prob_threshold
      new_landmark = LandmarkEKF()
      new_landmark.updateNewLandmark(pose_est, pose_cov_est, meas, meas_cov)
      self.landmarks[self.next_idx]=new_landmark
      self.next_idx+=1
    else:
      to_remove = []
      for (idx, landmark), prob_associate in zip(self.landmarks.items(),prob_associate_ls):
        if idx == observed_land_idx:
          self.weight = landmark.updateObserved()
        else:
          keep = landmark.updateUnobserved()
          if not keep:
            to_remove.append(idx)
      for idx in to_remove:
        self.landmarks.pop(idx)
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

    pose_noise = self.random.normal(0, self.pose_cov, 7)
    # rand_vx, rand_vy, rand_prev_vx, rand_prev_vy, rand_theta, rand_omega = self.random.normal(0, self.pose_cov, 7)
    # rand_vx, rand_vy = self.random.normal(0,self.v_sigma,2)
    # rand_prev_vx, rand_prev_vy = self.random.normal(0,self.v_sigma,2)
    # rand_theta = self.random.normal(0, self.theta_sigma,1)
    # rand_omega = self.random.normal(0, self.omega_sigma,1)
    
    x = prev_x + vx * dt
    y = prev_y + vy * dt

    theta = theta_imu

    vx = v * np.cos(theta)
    vy = v * np.sin(theta)

    omega = (theta - prev_theta) / dt # NOT SURE

    theta_p = prev_theta

    pose = np.array([x,y,theta, vx, vy, omega, theta_p]) + pose_noise

    '''
    dx = np.array([1,0,0,dt,0,0,0])
    dy = np.array([0,1,0,0,dt,0,0])
    dtheta = np.array([0,0,0,0,0,0,0])
    dvx = np.array([0,0,-v*np.sin(prev_theta) ,0,0,0,0])
    dvy = np.array([0,0,-v*np.cos(prev_theta) ,0,0,0,0])
    domega = np.array([0,0,-dt,0,0,0,0]) # NOT SURE
    dtheta_p =  np.array([0,0,1,0,0,0])

    motion_jocabian = np.array([dx,dy,dtheta, dvx, dvy, domega, dtheta_p])
    '''

    # pose_cov = np.eyes([rand_vx*dt, rand_vy*dt])
    # pose_cov = np.eyes([0, 0, self.theta_sigma, self.v_sigma, self.v_sigma, ])
    return pose
