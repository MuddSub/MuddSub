from dataclasses import dataclass
import numpy as np

@dataclass
class EFK:
  mean: np.ndarray #landmark mean
  cov: np.ndarray #landmark cov

  innovation: np.ndarray
  meas_jac_pose: np.ndarray
  meas_jac_land: np.ndarray
  inv_pose_cov: np.ndarray
  Q: np.ndarray
  inv_Q: np.ndarray

  exist_log: float = 0


