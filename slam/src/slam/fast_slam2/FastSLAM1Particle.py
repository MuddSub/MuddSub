import numpy as np
import scipy.stats
from scipy.linalg import sqrtm
from collections import namedtuple
from slam.fast_slam2.Models import _EKF, Meas, LandmarkConstants
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D
from slam.fast_slam2.FastSLAM2Particle import FastSLAM2Particle
from typing import List

class FastSLAM1Particle(FastSLAM2Particle):
  '''
  Class representing a particle in FastSLAM2
  '''
  def __init__(self, robot_physics: RobotPhysics2D, particle_id=None, **kwargs):
    # random=None, map=None, num_landmarks_fixed=False, new_landmark_threshold=0.1, id=None
    super().__init__(robot_physics, particle_id, **kwargs)

  def _update_dependencies_on_landmark(self, landmark: _EKF,  pose_mean, pose_cov, meas):
    est_meas_data = self._robot_physics.compute_meas_model(pose_mean, landmark.mean)
    _, landmark.meas_jac_land = self._robot_physics.compute_meas_jacobians(pose_mean, landmark.mean)
    landmark.innovation = meas.data - est_meas_data

    landmark.Q = meas.cov + landmark.meas_jac_land @ landmark.cov @ landmark.meas_jac_land.T
    landmark.Q_inv = np.linalg.inv(landmark.Q)
  def _update_pose_distribution(self, landmark, pose_mean, pose_cov):
    landmark.pose_cov = self.pose_cov
    landmark.pose_mean = self.pose
    landmark.sampled_pose = self._random.multivariate_normal(pose_mean, pose_cov)

  def _update_landmark_association_prob(self, meas, landmark):    
    exponent = -0.5 * (landmark.innovation).T @ landmark.Q_inv @ landmark.innovation
    inv_sqrt_of_two_pi_det_Q = (np.linalg.det(2 * np.pi * landmark.Q)) ** -0.5
    landmark.association_prob = inv_sqrt_of_two_pi_det_Q * np.exp(exponent)

  def _update_observed_landmark(self,landmark, pose_cov, meas):
    landmark.exist_log += self._landmark_constants.exist_log_inc
    # compute joint distribution (Kalman gain)
    prev_landmark_cov = landmark.cov
    K =  prev_landmark_cov @ landmark.meas_jac_land.T @ landmark.Q_inv 
    I = np.eye(landmark.mean.shape[0])
    # update landmark
    landmark.mean = landmark.mean + K @ landmark.innovation
    landmark.cov = ( I - K @ landmark.meas_jac_land) @ prev_landmark_cov
    # Compute particle weight
    landmark.particle_weight = landmark.association_prob
    self._log('update observed landmark has particle pose',self.pose)