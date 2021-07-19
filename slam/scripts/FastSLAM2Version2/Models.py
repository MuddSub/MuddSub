from dataclasses import dataclass
import numpy as np
from collections import namedtuple
LandmarkConstants = namedtuple('LandmarkConstants',['exist_log_inc','exist_log_dec'])

@dataclass 
class FastSLAM2Parameters:
  num_particles: int

  is_landmarks_fixed: bool
  new_landmark_threshold: float

  initial_landmarks: dict()

@dataclass
class _EKF: # a landmark class
  # main values
  name: str = ''
  mean: np.ndarray = np.array([]) #landmark mean
  cov: np.ndarray = np.array([]) #landmark cov

  # associated values
  sampled_pose: np.ndarray = np.array([])
  pose_mean: np.ndarray  = np.array([])
  pose_cov: np.ndarray = np.array([])

  innovation: np.ndarray  = np.array([])
  meas_jac_pose: np.ndarray  = np.array([])
  meas_jac_land: np.ndarray  = np.array([])
  inv_pose_cov: np.ndarray  = np.array([])
  Q: np.ndarray  = np.array([])
  inv_Q: np.ndarray  = np.array([])

  association_prob: float = 0
  exist_log: float = 0
  particle_weight: float = 0

_MEAS = namedtuple('MEAS', ['meas_data', 'meas_cov', 'sensor_constraints', 'correspondence'])
class MEAS(_MEAS):
  '''
  meas_data:            A tuple or namedtuple containing the measurement data, i.e. range and bearing or just bearing depending on the type of data the robot expects to receive.
  meas_cov:             A square numpy array that is the covariance matrix for the measurement data
  sensor_constraints:   A tuple or namedtuple containing sensor constraints, such as maximum range and viewing angle, or just viewing angle depending on the sensor the data is collected by.
  data_association:     A hashable ??????
  '''
  pass
