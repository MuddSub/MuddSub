from abc import ABC, abstractmethod
import numpy as np
class RobotPhysicsBase(ABC):
    '''
    Provides Robot Physics Computatons 
    '''
    def __init__(self, random):
        self._random = random

    @abstractmethod
    def compute_meas_model(self):
        pass
    
    @abstractmethod
    def compute_meas_jacobians(self):
        pass
    
    @abstractmethod
    def compute_inverse_meas_model(self):
        pass

    @abstractmethod
    def compute_motion_model(self):
        pass

    @abstractmethod
    def is_landmark_in_range(self, pose, landmark_pos, sensor_constraints):
        pass
  
    def compute_sample_pose(self, pose, pose_cov):
      return self._random.multivariate_normal(pose, pose_cov)