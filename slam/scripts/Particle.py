import numpy as np

class EKF():
  def __init__(self):
    self.stateEstimate = None
    self.stateCovariance = None
    pass

  def computeMeasJacobianWRTLandmark(range, bearing, robotState):
    pass
  
  def computeMeasJacobianWRTPose(range, bearing, robotState):
    pass

  def computeMotion():
    pass
  # prob need more computation 
  
  def correct(range, bearing, robotState):
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
    self.random_generator = np.random.default_rng(seed)
    
    self.weight = 1

    self.robot_pose = np.zeros(7)

    self.num_landmarks = params['num_landmarks']
    self.landmarks = dict() # landmark_id: EKF


    self.velocity_sigma = params['velocity_sigma']
    # more! 

    pass

  def propagateMotion(self,velocity_meas, theta_meas, dt):
    pass

  def correct(self,time, landmark, range, bearing):
    pass

