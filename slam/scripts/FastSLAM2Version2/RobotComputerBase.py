from abc import ABC, abstractmethod
import numpy as np
class RobotComputerBase(ABC):
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

    def is_landmark_in_range(self, measurement, acceptable_range):
        return measurement < acceptable_range
  
    def compute_noisy_pose(self,pose,pose_cov):
      return self._random.multivariate_normal(pose, pose_cov)
