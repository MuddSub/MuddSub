from abc import ABC, abstractmethod
import numpy as np
from slam.Utils import wrap_to_pi
from slam.robot_physics.RobotPhysicsBase import RobotPhysicsBase

class RobotPhysics2D(RobotPhysicsBase):
  '''
  Describes the physics for a robot in a 2D environment controlled via velocity and angular velocity commands.
  The robot has a single camera with a max range and a field of view that measures range and bearing to landmarks.
  '''
  def __init__(self, random):
    super().__init__(random)

  def compute_meas_model(self, pose, landmark_mean):
    '''
    Compute the measurement for a landmark given robot pose and landmark mean. 
    '''
    x, y, theta = pose
    lx, ly = landmark_mean

    range_est = ((lx - x) ** 2 + (ly - y) ** 2) ** 0.5
    bearing_est = wrap_to_pi(np.arctan2((ly - y), (lx - x)) - theta) #TODO make imported

    return np.array([range_est, bearing_est])
  
  def compute_meas_jacobians(self, pose, landmark_mean):
    '''
    Compute a measurement Jacobian with respect to pose, and 
    a measurement Jacobian with respect to landmark
    '''
    lx, ly = landmark_mean
    x, y, theta, = pose
    
    range_est_sqr = ((lx-x)**2+(ly-y)**2)
    range_est = range_est_sqr**.5

    meas_jac_pose = np.zeros((2,3))
    
    meas_jac_pose[0, 0] = -(lx-x) / range_est
    meas_jac_pose[0, 1] = -(ly-y) / range_est
    meas_jac_pose[1, 0] = (ly-y) / range_est_sqr
    meas_jac_pose[1, 1] = -(lx-x) / range_est_sqr
    meas_jac_pose[1, 2] = -1
    meas_jac_land = -1 * meas_jac_pose[:2, :2]
    
    return meas_jac_pose, meas_jac_land
    
  def compute_inverse_meas_model(self,pose,meas):
    '''
    Compute a landmark position given robot pose and measurement. 
    '''
    x, y, theta = pose
    range_meas, bearing_meas = meas
    lx = x + range_meas * np.cos(theta + bearing_meas)
    ly = y + range_meas * np.sin(theta + bearing_meas)
    return np.array([lx, ly])
  
  def compute_motion_model(self, pose, control, dt):
    '''
    Compue robot's new pose given current pose, control, and time. 
    '''
    v, w = control
    if -1e-10 <= w <= 1e-10:
      w = 1e-10

    x, y, theta = pose
    next_theta = wrap_to_pi(theta + w * dt)
    next_x = x + v/w * (-np.sin(theta) + np.sin(next_theta))
    next_y = y + v/w * ( np.cos(theta) - np.cos(next_theta))
    return np.array([next_x, next_y, next_theta])
  
  def is_landmark_in_range(self, pose, landmark_pos, sensor_limits):
    '''
    Check whether the given landmark position is in range and in the field of view of the robot's camera 
    given a list of sensor limits. 
    '''
    # Unpack data
    x, y, theta = pose
    lx, ly = landmark_pos

    # Filter by range
    diff_x = lx - x
    diff_y = ly - y
    dist = (diff_x * diff_x + diff_y * diff_y)**.5   # Distance between the robot and the landmark
    
    # Filter by field of view
    unit_vec = (np.cos(theta), np.sin(theta))   # Compute the unit vector in the robot's direction
    dot = (diff_x * unit_vec[0] + diff_y * unit_vec[1]) / dist  # Compute the cosine of the angle between the robot's direction vector and the landmark using the dot product

    for sensor_limit in sensor_limits:
      max_range, fov = sensor_limit
      dot_range = np.cos(fov / 2) # This defines the range of cosine values between which the landmark falls in the robot's fov
      if dist <= max_range and dot <= dot_range:
        return True
    return False
