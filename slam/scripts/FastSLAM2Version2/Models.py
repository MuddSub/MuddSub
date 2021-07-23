from dataclasses import dataclass
import numpy as np
from collections import namedtuple

@dataclass
class LandmarkConstants:
  exist_log_inc: float = 0.1
  exist_log_dec: float = 0.1
  new_landmark_threshold: float = 0.1

@dataclass
class FastSLAM2Parameters:
  num_particles: int
  is_landmarks_fixed: bool
  initial_landmarks: dict

@dataclass
class _EKF: # a landmark class
  # main values
  name: str = ''
  mean: np.ndarray = None #landmark mean
  cov: np.ndarray = None #landmark cov

  # associated values
  sampled_pose: np.ndarray = None
  pose_mean: np.ndarray = None
  pose_cov: np.ndarray = None

  innovation: np.ndarray = None
  meas_jac_pose: np.ndarray = None
  meas_jac_land: np.ndarray = None
  inv_pose_cov: np.ndarray = None
  Q: np.ndarray = None
  inv_Q: np.ndarray = None

  association_prob: float = None
  exist_log: float = None
  particle_weight: float = None

_Meas = namedtuple("Meas", ["meas_data", "meas_cov", "sensor_constraints", "data_association"])
class Meas(_Meas):
  '''
  meas_data:            A tuple or namedtuple containing the measurement data, i.e. range and bearing or just bearing depending on the type of data the robot expects to receive.
  meas_cov:             A square numpy array that is the covariance matrix for the measurement data
  sensor_constraints:   A tuple or namedtuple containing sensor constraints, such as maximum range and viewing angle, or just viewing angle depending on the sensor the data is collected by.
  data_association:     A hashable to identify an obstacle
  '''
  pass