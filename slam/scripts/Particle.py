import numpy as np
from scipy.linalg import sqrtm
# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)


'''
s: pose --> robot pose (7 states)
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
P: pose_cov: measurement covariance

tau: land_exist_log --> log odds of the probability that the landmark exists
rho+: land_exist_log_inc --> log odds value to add to land_exist_log when positive evidence of the landmark is seen
rho-: land_exist_log_neg --> log odds value to subtract to land_exist_log when negative evidence of the landmark is seen

Q: Q --> fused measurement covariance. meas_cov + meas_jac_land @ land_cov @ meas_jac_land
Q: Q_inv --> Q inverse
K: K --> Kalman gain

w: weight 

p_nt: prob_match --> probability of landmark associates with given measurement
'''

class LandmarkEKF():
  def __init__(self, landmark_state_estimate, landmark_state_convariance, seed):
    self.prev_state_estimate = np.ones(2)
    self.prev_state_covariance = np.ones((2,2))
    self.Q_inverse = None 

    self.state_estimate = np.ones(2)
    self.state_covariance = np.ones((2,2))
    
    self.random = np.random.default_rng(seed)
    self.prob_data_association = 0
    
    self.land_exist_log = 0
    self.land_exist_log_threshold = self.logOdds(0.5)
    self.land_exist_log_inc = self.logOdds(0.8) # 0.8 works out to around 1.38, rho positive in the FastSLAM 2.0 algorithm
    self.land_exist_log_dec = self.logOdds(0.2) # 0.2 works out to around -1.38, rho negative in the FastSLAM 2.0 algorithm


  def logOdds(self, probability):
    return np.log(probability / (1 - probability))

  def computeMeasurementModel(self, est_robot_state):
    if len(est_robot_state)==7:
      x,y,theta, vx, vy, omega, theta_p = est_robot_state
    else:
      x, y = est_robot_state

    lx, ly = self.prev_state_estimate

    est_range = ((lx-x)**2+(ly-y)**2)**.5
    est_bearing = np.arctan2((ly-y)/(lx-x))-theta

    return [est_range, est_bearing]

  def computeMeasJacobians(self, est_robot_state):
    lx, ly = self.prev_state_estimate
    x,y,theta, vx, vy, omega, theta_p = est_robot_state
    est_range_sqr = ((lx-x)**2+(ly-y)**2)
    est_range = est_range_sqr**.5
    Gs = np.zeros((2,7))

    Gs[0,0] = -(lx-x) / est_range
    Gs[1,0] = -(ly-y) / est_range
    Gs[0,1] = (ly-y) / est_range_sqr
    Gs[0,0] = -(lx-x) / est_range_sqr
    self.Gs = Gs
    self.Gtheta = -1 * Gs
    return self.Gs, self.Gtheta
    
  '''  
  def computeMeasJacobianWRTLandmark(self, est_robot_state):
    return self.Gtheta
    
  def computeMeasJacobianWRTPose(self, est_robot_state):
    return self.Gs
  '''

  def dataAssociation(self, est_robot_state, motion_convariance, measurements, meas_covariance):
    est_robot_pose = np.array([est_robot_state[0],est_robot_state[1]])
    meas_range, meas_bearing = measurements   
    

    est_range, est_bearing = self.computeMeasurementModel(est_robot_state)     
    
    Gs, Gtheta = self.computeMeasJacobians( est_robot_state)

    meas_diff = np.array([meas_range - est_range, meas_bearing - est_bearing])
    self.meas_diff = meas_diff
    
    Q = meas_covariance + Gtheta @ self.prev_state_covariance @ Gtheta.T
    self.Q_inverse = np.linalg.inv(Q)
    Q_inverse = self.Q_inverse
    
    motion_convariance_inverse = np.linalg.inv(motion_convariance)
    
    sampling_convariance = np.linalg.inv(Gs @ Q_inverse @ Gs + motion_convariance_inverse)
    sampling_mean = sampling_convariance @ Gs.T @ Q_inverse @ meas_diff +est_robot_pose

    sampled_robot_pose = sampling_mean + self.random.normal(0,sampling_convariance)
  
    Q_inverse_sqrt_root = np.linalg.inv(sqrtm(np.abs(2*np.pi*Q)))

    improved_range, improved_bearing = np.array(self.computeMeasurementModel(sampled_robot_pose))
    
    improved_diff = np.array([improved_range - est_range, improved_bearing - est_bearing])
    
    exponent = -1.0/2*(improved_diff).T @ Q_inverse @ (improved_diff)
    self.prob_data_association = Q_inverse_sqrt_root * np.exp(exponent)
    
    return self.prob_data_association


  def motion_and_correct(self, motion_convariance, meas_covariance, range, bearing, robotState):

    Gtheta, Gs = self.Gtheta, self.Gs

    self.land_exist_prob += self.land_exist_log_inc

    K =  self.prev_state_covariance @ self.Gtheta.T @ self.Q_inverse 

    self.state_estimate = self.prev_state_estimate + K @ self.meas_diff

    self.state_covariance = (np.eyes(7) - K @ Gtheta) @ self.prev_state_covariance

    L = Gs @ motion_convariance @ Gs.T + Gtheta @ self.prev_state_covariance @ Gtheta.T + meas_covariance
    L_inv = np.linalg.inv(L)
    exponent = -1.0/2*(self.meas_diff ).T @ L_inv @ (self.meas_diff )
    w = np.linalg.inv(sqrtm(np.abs(2*np.pi* L ))) * np.exp(exponent)

    return w

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
  def __init__(self, state,params, particle_id, seed):
    self.id = particle_id
    self.random = np.random.default_rng(seed)
    
    self.weight = 1

    self.robot_state = np.zeros(7)
    self.robot_state_hist = []

    self.num_landmarks = params['num_landmarks']
    self.landmarks = dict() # landmark_id: EKF


    self.v_sigma = params['v_sigma']
    self.omega_sigma = params['omega_sigma']
    self.theta_sigma = params['theta_sigma']
    # more! 

    
    pass

  def propagateMotion(self, control, measurement, meas_covariance, landmark_key, dt):
    self.robot_state_hist.append(self.robot_state)
    est_robot_state, motion_convariance = self.computeMotionModel(self.robot_state, control, dt)
     
    # we might not need this -- we have accurate data association 
    prob_data_association_ls = []
    for landmark in self.landmarks:
      prob_data_association = landmark.dataAssociation(est_robot_state, motion_convariance, measurement, meas_covariance)
      prob_data_association_ls.append(prob_data_association)
    landmark_key = np.argmax(np.array(prob_data_association_ls))
    updated_landmark = self.landmarks[landmark_key]
    updated_landmark.motion_and_correct(est_robot_state, motion_convariance, measurement, meas_covariance)

    # do correct 

  # set a method that deals with unknown landmark 

  def correct(self,landmark_key = None, new_landmark=False):
    if new_landmark:
      landmark_key = len(self.landmarks)
      landmark = LandmarkEKF()
    else:
      pass
      # particle correction 
    # find nearby landmark 
    # and downweight 
    
  def computeMotionModel(self, prev_pose, control, dt):
    vx, vy, theta_imu, omega_imu = control
    prev_x, prev_y, prev_theta, prev_vx, prev_vy, prev_omega, prev_v_p = prev_pose
    
    v = (vx**2+vy**2)**.5

    rand_vx, rand_vy = self.random.normal(0,self.v_sigma,2)
    rand_prev_vx, rand_prev_vy = self.random.normal(0,self.v_sigma,2)
    rand_theta = self.random.normal(0, self.theta_sigma,1)
    rand_omega = self.random.normal(0, self.omega_sigma,1)
    
    x = prev_x + (vx+rand_vx) * dt  
    y = prev_y + (vy+rand_vy) * dt 

    theta = theta_imu + rand_theta

    vx = v * np.cos(theta) + rand_prev_vx
    vy = v * np.sin(theta) + rand_prev_vy

    omega = (theta - prev_theta) / dt # NOT SURE

    theta_p = prev_theta

    est_state = (x,y,theta, vx, vy, omega, theta_p)

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
    motion_convariance = np.eyes([rand_vx*dt, rand_vy*dt])
    return est_state, motion_convariance
