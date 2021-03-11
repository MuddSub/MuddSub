import numpy as np

# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)

class LandmarkEKF():
  def __init__(self):
    self.state_estimate = None
    self.state_covariance = None
    
    pass

  def computeMeasJacobianWRTLandmark(self,range, bearing, robotState):
    pass
  
  def computeMeasJacobianWRTPose(self,range, bearing, robotState):
    pass


  def computeMeasurementModel(self, est_robot_pose):
    x,y,theta, vx, vy, w, theta_p = est_robot_pose

    lx, ly = self.state_estimate

    est_range = ((lx-x)**2+(ly-y)**2)**.5
    est_bearing = np.arctan2((ly-y)/(lx-x))-theta

    return est_range, est_bearing

  def computeSamplingDistribution(self):
    pass
  # prob need more computation 
  
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
    self.w_sigma = params['w_sigma']
    self.theta_sigma = params['theta_sigma']
    # more! 

    pass

  def propagateMotion(self,velocity_meas, theta_meas, dt):
    pass

  def correct(self,time, landmark, range, bearing):
    pass
  
  def computeMotionModel(self, prev_pose, control, dt):
    vx, vy, theta_imu, w_imu = control
    prev_x, prev_y, prev_theta, prev_vx, prev_vy, prev_w, prev_v_p = prev_pose
    
    v = (vx**2+vy**2)**.5

    rand_vx, rand_vy = self.random.normal(0,self.v_sigma,2)
    rand_prev_vx, rand_prev_vy = self.random.normal(0,self.v_sigma,2)
    rand_theta = self.random.normal(0, self.theta_sigma,1)
    rand_w = self.random.normal(0, self.w_sigma,1)
    
    x = prev_x + (vx+rand_vx) * dt  
    y = prev_y + (vy+rand_vy) * dt 

    theta = theta_imu + rand_theta

    vx = v * np.cos(theta) + rand_prev_vx
    vy = v * np.sin(theta) + rand_prev_vy

    w = None # not sure

    theta_p = prev_theta

    est_pose = (x,y,theta, vx, vy, w, theta_p)

    return est_pose
