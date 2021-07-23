import numpy as np
from scipy.linalg import sqrtm
from collections import namedtuple
from Models import _EKF, MEAS, LandmarkConstants
from PhysicsComputer2D import PhysicsComputer2D
from typing import List
class Particle():
  '''
  Class representing a particle in FastSLAM2
  '''
  def __init__(self, physics: PhysicsComputer2D, particle_id = None, **kwargs):
    # random=None, map=None, num_landmarks_fixed=False, new_landmark_threshold=0.1, id=None

    # Private variables from kwargs
    self._id = particle_id
    self._random = kwargs.get('random', np.random.default_rng())
    self._is_landmarks_fixed = kwargs.get('is_landmarks_fixed', False)
    self._new_landmark_threshold = kwargs.get('new_landmark_threshold', 0.1)

    # Private required variables 
    self._physics = physics

    # Private paramters
    self._landmark_constants = LandmarkConstants(exist_log_inc=.1,exist_log_dec=.1) 

    # Public variables
    self.weight = 0
    self.accumulated_weight = 0
    self.pose = np.copy(physics.initial_pose)
    self.pose_cov = np.copy(physics.default_pose_cov)
    self.landmarks = kwargs.get('initial_landmarks', {})

  def set_params(self, **kwargs):
    '''
    Set particle parameters
    '''
    self._is_landmarks_fixed = kwargs.get('is_landmarks_fixed', self._is_landmarks_fixed)
    self._new_landmark_threshold = kwargs.get('new_landmark_threshold', self._new_landmark_threshold)


  def _log(self, *msg):
    print('+++ Particle '+str(self._id)+':', *msg)
  

  def update_motion(self, control, dt):
    '''
    Update particle pose using motion model
    '''
    self._log('motion1',self.pose)
    self.pose = self._physics.compute_motion_model(self.pose, control, dt)
    self._log('motion2',self.pose)

  def update_meas(self, meas_ls: List[MEAS]):
    '''
    Identify current facing landmark
    Correct particle pose using measurements
    Compute particle weight
    '''
    # ++++++ Set up: sort meas_ls by range; initialize particle mean, covariance, and weight
    meas_ls.sort(key = lambda meas: meas.meas_data[0])  # meas_ls should be sorted by range.
    pose_mean, pose_cov = np.copy(self.pose),  np.copy(self.pose_cov)  
    self.weight = 1.0 
    # ++++++ No Measurment: sample the pose using motion model
    if len(meas_ls) == 0:
      self.pose = self._physics.compute_noisy_pose(pose_mean, pose_cov)
      return
    # ++++++ Access each measurement 
    curr_landmark = None
    # Iteratively update pose mean and pose covariance using each measurement of the same time step
    for meas in meas_ls:
      # ++++++ If unknown correspondence, perform data association for each landmark
      if meas.correspondence == None: 
        for land_idx, landmark in list(self.landmarks.items()):
          # +++ New particle pose is computed here
          self._update_dependencies_on_landmark(landmark, pose_mean, pose_cov, meas.meas_data, meas.meas_cov)
          self._log('meas 1',self.pose)
          self._update_pose_distribution(landmark, pose_mean, pose_cov)
          self._log('meas 2',self.pose)
          self._update_landmark_association_prob(landmark)
          self._log('meas 3',self.pose)
        if len(self.landmarks) > 0: # use maximum likelihood to select landmark
          curr_landmark = max(list(self.landmarks.values()), key = lambda landmark: landmark.association_prob)
      
        # No landmark or not confident: create new landmark 
        if curr_landmark == None \
          or (curr_landmark.association_prob < self._new_landmark_threshold and self._is_landmarks_fixed):
          curr_landmark = self.landmarks[len(self.landmarks) + 1 ] = \
            self._init_landmark(self._physics.compute_noisy_pose(pose_mean,pose_cov), pose_mean, pose_cov, meas.meas_data, meas.meas_cov)
          self._log('meas 4',self.pose)
        else:
          self._update_observed_landmark(curr_landmark, pose_cov, meas.meas_cov)
          self._log('meas 5',self.pose)
      # ++++++ If known correspondence
      else:
        if meas.correspondence in self.landmarks:
          # +++ New particle pose is computed here
          curr_landmark = self.landmarks[meas.correspondence]
          self._update_dependencies_on_landmark(curr_landmark, pose_mean, pose_cov, meas.meas_data, meas.meas_cov)
          self._log('meas 6',self.pose)
          self._update_pose_distribution(curr_landmark, pose_mean, pose_cov)
          self._log('meas 7',self.pose)
          self._update_observed_landmark(curr_landmark, pose_cov, meas.meas_cov)
          self._log('meas 8',self.pose)
        else: 
          curr_landmark = self.landmarks[meas.correspondence] = \
            self._init_landmark( self._physics.compute_noisy_pose(pose_mean,pose_cov), pose_mean, pose_cov, meas.meas_data, meas.meas_cov, meas.correspondence)
          self._log('meas 9',self.pose)
      # ++++++ Penalize landmarks that we expect to see   
      if self._is_landmarks_fixed:
        self._remove_some_unobserved_landmarks(curr_landmark, meas)
      # ++++++ Update particle properties
      self.weight *= curr_landmark.association_prob
      self.weight = max(self.weight, 1e-50)
      self.pose_mean = np.copy(curr_landmark.pose_mean)
      self._log('meas 10',self.pose)
      self.pose_cov = np.copy(curr_landmark.pose_cov)
      self.pose = np.copy(curr_landmark.sampled_pose)
      self._log('meas 11',self.pose)
  
  def _update_dependencies_on_landmark(self, landmark: _EKF,  pose_mean, pose_cov, meas_data, meas_cov):
    est_meas_data = self._physics.compute_meas_model(pose_mean, landmark.mean) # range_est, bearing_est
    landmark.meas_jac_pose, landmark.meas_jac_land = self._physics.compute_meas_jacobians(pose_mean, landmark.mean)
    landmark.innovation = meas_data - est_meas_data

    landmark.inv_pose_cov = np.linalg.inv(pose_cov) 
    landmark.Q = meas_cov + landmark.meas_jac_land @ landmark.cov @ landmark.meas_jac_land.T
    landmark.inv_Q = np.linalg.inv(landmark.Q)
    
  def _update_pose_distribution(self, landmark, pose_mean, pose_cov):
    landmark.pose_cov = np.linalg.inv(landmark.meas_jac_pose.T @ landmark.inv_Q @ landmark.meas_jac_pose + landmark.inv_pose_cov)
    landmark.pose_mean = pose_cov @ landmark.meas_jac_pose.T @ landmark.inv_Q @ landmark.innovation + pose_mean
    landmark.sampled_pose = self._random.multivariate_normal(pose_mean, pose_cov)

  def _update_landmark_association_prob(self, meas_data, landmark):
    improved_meas_data = self._physics.compute_meas_model(landmark.sampled_pose, landmark.mean) #range_improved, bearing_improved
    improved_innovation = meas_data - improved_meas_data
    
    exponent = -.5*(improved_innovation).T @ landmark.inv_Q @ improved_innovation
    two_pi_inv_Q_sqrt = (np.linalg.det(2*np.pi* landmark.Q)) ** -0.5
    landmark.association_prob = two_pi_inv_Q_sqrt * np.exp(exponent)

  def _update_observed_landmark(self,landmark, pose_cov, meas_cov):
    landmark.exist_log += self._landmark_constants.exist_log_inc
    # compute joint distribution (Kalman gain)
    K =  landmark.cov @ landmark.meas_jac_land.T @ landmark.inv_Q 
    I = np.eye(landmark.mean.shape[0])
    # update landmark
    landmark.mean = landmark.mean + K @ landmark.innovation
    prev_landmark_cov = landmark.cov
    landmark.cov = ( I - K @ landmark.meas_jac_land) @ prev_landmark_cov
    # Compute particle weight
    L = landmark.meas_jac_pose @ pose_cov @ landmark.meas_jac_pose.T \
         + landmark.meas_jac_land @ prev_landmark_cov @ landmark.meas_jac_land.T \
         + meas_cov 
    L_inv = np.linalg.inv(L)
    two_pi_L_inv_sqrt = (np.linalg.det(2*np.pi*L)) ** -0.5
    exponent = -.5* landmark.innovation.T @ L_inv @ landmark.innovation 
    landmark.particle_weight = two_pi_L_inv_sqrt * np.exp(exponent)

  def _init_landmark(self, sampled_pose,  pose_mean, pose_cov, meas_data, meas_cov, name = None):
    landmark = _EKF()
    landmark.name = name
    landmark.mean = self._physics.compute_inverse_meas_model(sampled_pose,meas_data)
    landmark.meas_jac_pose, landmark.meas_jac_land = self._physics.compute_meas_jacobians(sampled_pose, landmark.mean)
    landmark.cov = np.linalg.inv(landmark.meas_jac_land @ np.linalg.inv(meas_cov) @ landmark.meas_jac_land.T)
    
    landmark.exist_log = self._landmark_constants.exist_log_inc
    
    landmark.pose_mean = pose_mean
    landmark.pose_cov = pose_cov
    landmark.sampled_pose = self._physics.compute_noisy_pose(pose_mean, pose_cov)
    landmark.particle_weight = self._new_landmark_threshold
    return landmark

  def _is_unobserved_landmark_kept(self, landmark, sensor_constraints):
    # Update probability of the landmark existing based on whether it should have been measured

    if len(landmark.sampled_pose) ==  0:
      pass
    expected_meas_data = self._physics.compute_meas_model(landmark.sampled_pose, landmark.mean)
    if self._physics.is_landmark_in_range(expected_meas_data, sensor_constraints).all():
      landmark.exist_log -= self._landmark_constants.exist_log_dec
    return landmark.exist_log >= 0 # If the log odds probability falls below 0, we do not keep the landmark

  def _remove_some_unobserved_landmarks(self, curr_landmark, meas):
    '''
    Remove landmarks that we are supposed to see but did not see. 
    '''
    landmark_idxs_to_remove = []
    for landmark_idx, landmark in self.landmarks.items():
      if landmark_idx != curr_landmark.name: # Don't update observed landmark
        if not self._is_unobserved_landmark_kept(landmark, meas.sensor_constraints):
          landmark_idxs_to_remove.append(landmark_idx)
    
    for idx in landmark_idxs_to_remove: # Remove unobserved landmarks
      self.landmarks.pop(idx)