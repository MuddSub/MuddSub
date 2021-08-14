from dataclasses import dataclass
import numpy as np
from collections import namedtuple

@dataclass
class LandmarkConstants:
  """
  A dataclass to hold constants relating to
  landmark evidence used by the particles.

  Attributes
  ----------
  exist_log_inc : float
    The value to increment the log evidence by.
  exist_log_dec : float
    The value to decrement the log evidence by.
  new_landmark_threshold : float
    The threshold for a new landmark, as well
    as the initial evidence for new landmarks.
  """
  exist_log_inc: float = 0.1
  exist_log_dec: float = 0.1
  new_landmark_threshold: float = 0.1

@dataclass
class FastSLAM2Parameters:
  """
  A dataclass to hold the parameters used to
  initialize FastSLAM2.

  Attributes
  ----------
  num_particles : int
    The number of particles for the algorithm to use
  are_landmarks_fixed : bool
    If true, new landmarks cannot be added and current
    landmarks cannot be removed. Initial landmarks
    should be given if this is true.
  initial_landmarks : dict
    A dictionary containing landmarks to initialize each
    particle with. The dictionary keys are the landmark
    names, and the values are a tuple containing the
    initial landmark mean (numpy array) and landmark covariance
    (numpy array). If the initial covariance is None, it will 
    be set to a covariance matrix with 1e-5 as its diagonal
    entries.
  initial_pose : np.ndarray
    The initial robot pose
  initial_pose_cov : np.ndarray
    The initial robot pose covariance
  landmark_constants : LandmarkConstants
    The landmark constants used by the particles
  localization_only : bool = False
    If true, the algorithm performs particle filter localization
    only. This requires an initial map to be given.
  verbose : int = 0
    When verbose > 0, the FastSLAM2 class produces print statements.
    When verbose > 1, the Particle class produces print statements
  fast_slam_version : int = 2
    Which version of FastSLAM to use, either 1 or 2.
  """
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
  A namedtuple representing a measurement from one of
  the robot's sensors.

  Attributes
  ----------
  data:
    A tuple or namedtuple containing the measurement 
    data, i.e. range and bearing or just bearing depending 
    on the type of data the robot expects to receive.
  cov:
    A square numpy array that is the covariance matrix 
    for the measurement data.
  sensor_constraints:
    A tuple or namedtuple containing sensor constraints, 
    such as maximum range and viewing angle, or just 
    viewing angle depending on the sensor the data is 
    collected by.
  correspondence:    
    A hashable to identify an obstacle
  '''
  pass