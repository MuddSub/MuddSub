from dataclasses import dataclass
import numpy as np

@dataclass
class EFK:
  name: str 
  mean: np.ndarray #landmark mean
  cov: np.ndarray #landmark cov

  sampled_pose: np.ndarray
  pose_mean: np.ndarray
  pose_cov: np.ndarray

  innovation: np.ndarray
  meas_jac_pose: np.ndarray
  meas_jac_land: np.ndarray
  inv_pose_cov: np.ndarray
  Q: np.ndarray
  inv_Q: np.ndarray

  association_prob: float = 0
  exist_log: float = 0