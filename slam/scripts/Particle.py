import numpy as np
from scipy.linalg import sqrtm
# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)

class LandmarkEKF():
  def __init__(self, landmark_state_estimate, landmark_state_convariance, seed):
    self.prev_state_estimate = np.ones(2)
    self.prev_state_covariance = np.ones((2,2))
    
    self.state_estimate = np.ones(2)
    self.state_covariance = np.ones((2,2))
    
    self.random = np.random.default_rng(seed)
    

  def computeMeasurementModel(self, est_robot_state):
    if len(est_robot_state)==7:
      x,y,theta, vx, vy, omega, theta_p = est_robot_state
    else:
      x, y = est_robot_state

    lx, ly = self.prev_state_estimate

    est_range = ((lx-x)**2+(ly-y)**2)**.5
    est_bearing = np.arctan2((ly-y)/(lx-x))-theta

    return [est_range, est_bearing]

  def computeMeasJacobian(self, est_robot_state):
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
    
  '''  
  def computeMeasJacobianWRTLandmark(self, est_robot_state):
    return self.Gtheta
    
  def computeMeasJacobianWRTPose(self, est_robot_state):
    return self.Gs
  '''

  def computeSamplingDistribution(self, est_robot_state, motion_convariance, measurements, meas_covariance):
    est_robot_pose = np.array([est_robot_state[0],est_robot_state[1]])
    meas_range, meas_bearing = measurements   
    est_range, est_bearing = self.computeMeasurementModel(est_robot_state) 
    
    self.computeMeasJacobian( est_robot_state)
    
    Gtheta = self.Gtheta
    Gs = self.Gs
    
    Q = meas_covariance + Gtheta @ self.prev_state_covariance @ Gtheta.T
    Q_inverse = np.linalg.inv(Q)
    motion_convariance_inverse = np.linalg.inv(motion_convariance)
    
    sampling_convariance = np.linalg.inv(Gs @ Q_inverse @ Gs + motion_convariance_inverse)
    sampling_mean = sampling_convariance @ Gs.T @ Q_inverse @ (meas_range-est_range)+est_robot_pose

    sampled_robot_pose = sampling_mean + self.random.normal(0,sampling_convariance)
  
    Q_inverse_sqrt_root = np.linalg.inv(sqrtm(2*np.pi*Q))

    improved_range = np.array(self.computeMeasurementModel(sampled_robot_pose))

    exponent = -1.0/2*(meas_range-improved_range).T @ Q_inverse @ (meas_range-improved_range)
    prob_match = Q_inverse_sqrt_root * np.exp(exponent)
    return prob_match

  def correct(self,range, bearing, robotState):
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
  '''
  def __init__(self, state,params, particle_id, seed):
    self.id = particle_id
    self.random = np.random.default_rng(seed)
    
    self.weight = 1

    self.robot_pose = np.zeros(7)
    self.robot_pose_hist = []

    self.num_landmarks = params['num_landmarks']
    self.landmarks = dict() # landmark_id: EKF


    self.v_sigma = params['v_sigma']
    self.omega_sigma = params['omega_sigma']
    self.theta_sigma = params['theta_sigma']
    # more! 

    pass

  def propagateMotion(self,velocity_meas, theta_meas, dt):
    pass

  def correct(self,time, landmark, range, bearing):
    pass
  
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

    est_pose = (x,y,theta, vx, vy, omega, theta_p)

    return est_pose
