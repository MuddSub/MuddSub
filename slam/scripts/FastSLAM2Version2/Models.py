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
  landmark_constants: LandmarkConstants

@dataclass
class _EKF: # a landmark class
  # main values
  name: str
  mean: np.ndarray #landmark mean
  cov: np.ndarray #landmark cov

  # associated values
  sampled_pose: np.ndarray
  pose_mean: np.ndarray
  pose_cov: np.ndarray

  association_prob: float
  exist_log: float
  particle_weight: float

  meas_jac_pose: np.ndarray = None
  meas_jac_land: np.ndarray = None

  # Don't need to be initialized
  inv_pose_cov: np.ndarray = None
  Q: np.ndarray = None
  Q_inv: np.ndarray = None
  innovation: np.ndarray = None

_Meas = namedtuple("Meas", ["meas_data", "meas_cov", "sensor_constraints", "correspondence"])
class Meas(_Meas):
  '''
  meas_data:            A tuple or namedtuple containing the measurement data, i.e. range and bearing or just bearing depending on the type of data the robot expects to receive.
  meas_cov:             A square numpy array that is the covariance matrix for the measurement data
  sensor_constraints:   A tuple or namedtuple containing sensor constraints, such as maximum range and viewing angle, or just viewing angle depending on the sensor the data is collected by.
  correspondence:       A hashable to identify an obstacle
  '''
  pass