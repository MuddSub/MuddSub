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
  are_landmarks_fixed: bool
  initial_landmarks: dict
  initial_pose: np.ndarray
  initial_pose_cov: np.ndarray
  landmark_constants: LandmarkConstants
  localization_only: bool = False
  verbose: int = 0 # To make FastSlam2 verbose, use 1. To make Particle.py verbose, use 2. 
  fast_slam_version: int = 2

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

_Meas = namedtuple("Meas", ["data", "cov", "sensor_constraints", "correspondence"])
class Meas(_Meas):
  '''
  data:                 A tuple or namedtuple containing the measurement data, i.e. range and bearing or just bearing depending on the type of data the robot expects to receive.
  cov:                  A square numpy array that is the covariance matrix for the measurement data
  sensor_constraints:   A tuple or namedtuple containing sensor constraints, such as maximum range and viewing angle, or just viewing angle depending on the sensor the data is collected by.
  correspondence:       A hashable to identify an obstacle
  '''
  pass