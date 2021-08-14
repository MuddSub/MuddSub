import numpy as np
import scipy.stats
from scipy.linalg import sqrtm
from collections import namedtuple
from slam.fast_slam2.Models import _EKF, Meas, LandmarkConstants
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D
from typing import List 
from slam.Utils import safe_inverse

class FastSLAM2Particle():
  '''
  Class representing a particle in FastSLAM2
  '''
  def __init__(self, robot_physics: RobotPhysics2D, particle_id=None, **kwargs):
    # Private variables from kwargs
    self._id = particle_id
    self._random = kwargs.get('random', np.random.default_rng())
    self._are_landmarks_fixed = kwargs.get('are_landmarks_fixed', False)
    self._landmark_constants = kwargs.get('landmark_constants', LandmarkConstants())
    self._verbose = kwargs.get('verbose', 0)

    # Private required variables
    self._robot_physics = robot_physics

    # Public variables
    self.weight = 0
    self.accumulated_weight = 0
    self.pose = np.copy(kwargs['initial_pose'])
    self.pose_cov = np.copy(kwargs['initial_pose_cov'])
    self.landmarks = {}
    initial_landmarks = kwargs.get('initial_landmarks', {})

    # Initialize landmarks that are passed in
    for name, landmark in initial_landmarks.items():
      mean, cov = landmark
      if cov is None:
        cov = np.diag([1e-5] * mean.shape[0])
      # sampled_pose = self._robot_physics.compute_sample_pose(self.pose, self.pose_cov)
      self.landmarks[name] = self._init_landmark(str(name), mean, cov)

  def set_params(self, **kwargs):
    '''
    Set particle parameters
    '''
    # Private variables from kwargs that can be changed during runtime
    self._landmark_constants = kwargs.get('landmark_constants', LandmarkConstants())

  def _log(self, *msg):
    if self._verbose >= 2:
      print('Particle ' + str(self._id) + ':', *msg)
  
  def update_motion(self, control, dt):
    '''
    Update particle pose using motion model
    '''
    self._log('Update motion')
    self._log('before motion - particle pose',self.pose)
    self.pose = self._robot_physics.compute_motion_model(self.pose, control, dt)
    self._log('after motion - particle pose',self.pose)
    
  def update_meas(self, meas_ls: List[Meas]):
    '''
    Identify current facing landmark
    Correct particle pose using measurements
    Compute particle weight
    '''
    self._log('Update Measurement')
    self._log('before measurement - particle pose',self.pose)
    # ++++++ Set up: sort meas_ls by range; initialize particle mean, covariance, and weight
    meas_ls.sort(key = lambda meas: meas.data[0])  # meas_ls should be sorted by range.
    pose_mean, pose_cov = np.copy(self.pose), np.copy(self.pose_cov)
    self.weight = 1.0

    # ++++++ No Measurment: sample the pose using motion model
    if len(meas_ls) == 0:
      self.pose = self._robot_physics.compute_sample_pose(pose_mean, pose_cov)
      self._log('no measurement - particle pose',self.pose)
      return

    # ++++++ Access each measurement 
    observed_landmark = None
    observed_landmark_names = []
    meas_limits = []
    
    # Iteratively update pose mean and pose covariance using each measurement of the same time step
    for meas in meas_ls:
      # ++++++ If unknown correspondence, perform data association for each landmark 
      if meas.correspondence is None:
        for land_idx, landmark in list(self.landmarks.items()):
          # +++ New particle pose is computed here
          self._update_dependencies_on_landmark(landmark, pose_mean, pose_cov, meas)
          self._update_pose_distribution(landmark, pose_mean, pose_cov)
          self._update_landmark_association_prob(landmark)
          
        if len(self.landmarks) > 0: # use maximum likelihood to select landmark
          observed_landmark = max(list(self.landmarks.values()), key = lambda landmark: landmark.association_prob)
      
        # No landmark or not confident: create new landmark 
        if observed_landmark is None or ( \
              observed_landmark.association_prob < self._landmark_constants.new_landmark_threshold and \
              not self._are_landmarks_fixed \
          ):
          key = len(self.landmarks) + 1
          observed_landmark = self.landmarks[key] = \
            self._init_landmark_with_meas(pose_mean, pose_cov, meas, name=str(key))

        else:
          self._update_observed_landmark(observed_landmark, pose_cov, meas)  

      # ++++++ If known correspondence
      else:
        if meas.correspondence in self.landmarks:
          # +++ New particle pose is computed here
          observed_landmark = self.landmarks[meas.correspondence]
          self._update_dependencies_on_landmark(observed_landmark, pose_mean, pose_cov, meas)
          self._update_pose_distribution(observed_landmark, pose_mean, pose_cov)
          self._update_observed_landmark(observed_landmark, pose_cov, meas)
        else:
          observed_landmark = self.landmarks[meas.correspondence] = \
            self._init_landmark_with_meas(pose_mean, pose_cov, meas, name=str(meas.correspondence))

      # ++++++ Update particle properties
      self.weight *= observed_landmark.particle_weight
      self.weight = max(self.weight, 1e-50)
      self.pose_mean = np.copy(observed_landmark.pose_mean)
      self.pose_cov = np.copy(observed_landmark.pose_cov)
      self.pose = np.copy(observed_landmark.sampled_pose)
      observed_landmark_names.append(observed_landmark.name)
      meas_limits.append(meas.sensor_constraints)
      self._log('after measurement - particle pose',self.pose)
    
    # ++++++ Penalize landmarks that we expect to see
    if not self._are_landmarks_fixed:
      # +++ We will check all landmarks excepted observed_landmark 
      # +++ against a measurement's limits in ranges and bearings
      # +++ and penalize a landmark that is not seen.  
      self._update_unobserved_landmarks(observed_landmark_names, meas_limits)
    self._log('final after measurement - particle pose',self.pose)
  
  def update_meas_localization_only(self, meas_ls: List[Meas]):
    '''
    A measurement update for particle filter localization only.
    Currently assumes known correspondences.
    '''
    self.weight = 1.0

    # If there are no measurements, sample the pose according to the default pose covariance and the pose mean computed by the motion model
    if len(meas_ls) == 0:
      self.pose = self._robot_physics.compute_sample_pose(self.pose, self.pose_cov)
      return
    
    # Compute the particle's weight
    for meas in meas_ls:
      observed_landmark = self.landmarks[meas.correspondence]
      est_meas_data = self._robot_physics.compute_meas_model(self.pose, observed_landmark.mean)
      self.weight *= scipy.stats.multivariate_normal.pdf(est_meas_data, mean=meas.data, cov=meas.cov)

    # Bound the weight above a tiny number
    self.weight = max(self.weight, 1e-50)

  def _update_dependencies_on_landmark(self, landmark: _EKF,  pose_mean, pose_cov, meas):
    est_meas_data = self._robot_physics.compute_meas_model(pose_mean, landmark.mean)
    landmark.meas_jac_pose, landmark.meas_jac_land = self._robot_physics.compute_meas_jacobians(pose_mean, landmark.mean)
    landmark.innovation = meas.data - est_meas_data
    landmark.inv_pose_cov = safe_inverse(pose_cov, log=self._log, msg='Landmark pose cov is singular.')
    landmark.Q = meas.cov + landmark.meas_jac_land @ landmark.cov @ landmark.meas_jac_land.T
    landmark.Q_inv = safe_inverse(landmark.Q, log=self._log, msg='Q is singular.')
    
  def _update_pose_distribution(self, landmark, pose_mean, pose_cov):
    landmark.pose_cov = safe_inverse(landmark.meas_jac_pose.T @ landmark.Q_inv @ landmark.meas_jac_pose + landmark.inv_pose_cov, log=self._log, msg='Matrix used to compute pose is singular.')
    landmark.pose_mean = pose_cov @ landmark.meas_jac_pose.T @ landmark.Q_inv @ landmark.innovation + pose_mean
    landmark.sampled_pose = self._random.multivariate_normal(pose_mean, pose_cov)

  def _update_landmark_association_prob(self, meas, landmark):
    improved_meas_data = self._robot_physics.compute_meas_model(landmark.sampled_pose, landmark.mean) #range_improved, bearing_improved
    improved_innovation = meas.data - improved_meas_data
    
    exponent = -0.5 * (improved_innovation).T @ landmark.Q_inv @ improved_innovation
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
    L = landmark.meas_jac_pose @ pose_cov @ landmark.meas_jac_pose.T \
         + landmark.meas_jac_land @ prev_landmark_cov @ landmark.meas_jac_land.T \
         + meas.cov
    L_inv = safe_inverse(L, log=self._log, msg='L is singular.')
    inv_sqrt_of_two_pi_det_L = (np.linalg.det(2 * np.pi * L)) ** -0.5
    exponent = -0.5 * landmark.innovation.T @ L_inv @ landmark.innovation 
    landmark.particle_weight = inv_sqrt_of_two_pi_det_L * np.exp(exponent)

  def _init_landmark(self, name, mean, cov, **kwargs):
    landmark = _EKF(
      name = name, 
      mean = mean, 
      cov = cov,
      sampled_pose = kwargs.get('sampled_pose', self._robot_physics.compute_sample_pose(self.pose, self.pose_cov)),
      pose_mean = kwargs.get('pose_mean', self.pose),
      pose_cov = kwargs.get('pose_cov', self.pose_cov),
      association_prob = self._landmark_constants.new_landmark_threshold,
      exist_log = self._landmark_constants.exist_log_inc,
      particle_weight = self._landmark_constants.new_landmark_threshold
    )
    return landmark

  def _init_landmark_with_meas(self, pose_mean, pose_cov, meas, name = None):
    # Sample the particle's pose
    sampled_pose = self._robot_physics.compute_sample_pose(pose_mean, pose_cov)

    # Use the measurement data to compute the landmark's mean and covariance
    mean = self._robot_physics.compute_inverse_meas_model(sampled_pose, meas.data)
    meas_jac_pose, meas_jac_land = self._robot_physics.compute_meas_jacobians(sampled_pose, mean)
    cov = safe_inverse(meas_jac_land @ safe_inverse(meas.cov, log=self._log, msg='Measurement covariance is singular.') @ meas_jac_land.T, log=self._log, msg='Measurement jacobian with respect to the landmark is singular.')

    # Initialize and return landmark
    landmark = self._init_landmark(name, mean, cov, sampled_pose=sampled_pose, pose_mean=pose_mean, pose_cov=pose_cov)
    landmark.meas_jac_pose = meas_jac_pose
    landmark.meas_jac_land = meas_jac_land
    self._log('info: init landmark', landmark.name, landmark.association_prob,'with particle pose',sampled_pose)
    return landmark

  def _is_unobserved_landmark_kept(self, landmark):
    '''
    Update probability of the landmark existing based on whether it should have been measured.
    '''
    if landmark.sampled_pose is None:
      return True
    if self._robot_physics.is_landmark_in_range(landmark.sampled_pose, landmark.mean, meas_limits):
      landmark.exist_log -= self._landmark_constants.exist_log_dec
    return landmark.exist_log >= 0 # If the log odds probability falls below 0, we do not keep the landmark

  def _update_unobserved_landmarks(self, observed_landmark_names,meas_limits):
    '''
    Remove landmarks that we are supposed to see but did not see. 
    '''
    landmark_idxs_to_remove = []
    for landmark_idx, landmark in self.landmarks.items():
      if landmark_idx != observed_landmarks_names: # Don't update observed landmark
        if not self._is_unobserved_landmark_kept(landmark, meas_limits):
          landmark_idxs_to_remove.append(landmark_idx)
    
    for idx in landmark_idxs_to_remove: # Remove unobserved landmarks
      self.landmarks.pop(idx)