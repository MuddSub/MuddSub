import numpy as np
from scipy.linalg import sqrtm
from EKF import EKF
from collections import namedtuple

class Particle():
  '''
  Class representing a particle in FastSLAM2
  '''
  def __init__(self, robot, map=None, **kwargs):
    # random=None, map=None, num_landmarks_fixed=False, new_land_threshold=0.1, id=None

    # Private variables
    self._id = kwargs.get('id', None)
    self._random = kwargs.get('random', np.random.default_rng())
    self._fixed_num_landmarks = kwargs.get('fixed_num_landmarks', False)
    self._num_landmarks = 0
    self._new_land_threshold = kwargs.get('new_land_threshold', 0.1)
    self._pose_cov = np.copy(robot.default_pose_cov)
    self._landmarks = {}

    # to do: move this to util
    LandmarkConstants = namedtuple('LandmarkConstants', ['exist_log_inc', 'exist_log_dec'])
    self._landmark_constants = LandmarkConstants(.1, .1, 0)

    # Public variables
    self.weight = 0
    self.accumulated_weight = 0
    self.pose = None

  def motionUpdate(self, control, dt):
    # Pose estimate that is passed to each landmark EKF which uses it to calculate the sampling distribution and 
    # the probability of data assocation
    self.pose = self.robot.computeMotionModel(self.pose, control, dt)

  def measurementUpdate(self, meas_ls, meas_cov_ls, sensor_range_ls, sensor_bearing_ls, known_correspondences = False, correspondences = []):
    '''
    For best results, meas_ls should be sorted by range from closest to furthest.
    1. sample paricle pose per landmark 
    2. find the most probable observed landmark if we have unknown correspondences
    3. update particle pose using observed landmark 
    4. update landmark using sampled pose, pose mean, and pose covariance 
    '''
    # Initialize pose mean, pose covariance, and the particle's weight

    pose_mean = np.copy(self.pose)
    pose_cov = np.copy(self.default_pose_cov)
    self.weight = 1.0

    self.correpondences = correspondences

    # If there are no measurements, sample the pose according to the default pose covariance and the pose mean computed by the motion model
    if len(meas_ls) == 0:
      noise = self.computePoseNoise()
      self.pose = pose_mean + noise
      return

    # TODO: if a correspondence has low enough class probability, then we should do data association.

    # Iteratively update pose mean and pose covariance using each measurement of the same time step
    for meas_idx, (meas, meas_cov) in enumerate(zip(meas_ls, meas_cov_ls)):
      # Perform data association
      if not known_correspondences:
        prob_associate_ls = []
        land_idx_ls = [] # keep a separate list to enable mismatch betwee known and unknown correspondences
        for land_idx, landmark in self.landmarks.items():
          self.computeLandmarkSpecificVals(landmark, pose_mean, pose_cov, meas, meas_cov)
          prob_associate = self.updatePoseDistribution(landmark, pose_mean, pose_cov)
          prob_associate_ls.append(prob_associate)
          land_idx_ls.append(land_idx)

        # Pick landmark according to maximum likelihood
        ml_associated = 0  
        to_create_new_landmark = False
        if len(prob_associate_ls) > 0:
          ml_idx = np.argmax(prob_associate_ls)
          ml_associated = prob_associate_ls[ml_idx]
          
        # If we dont have any landmark, or assocation probability is really low, then we are seeing new landmarks 
        if len(prob_associate_ls) == 0 or (ml_associated < self.new_land_threshold and self.can_change_landmark):
          # Initialize new landmark
         
          observed_land_idx = len(land_idx_ls) + 1
          observed_landmark = LandmarkEKF(random=self.random) 
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.updateNewLandmark(self.pose + self.computePoseNoise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        else:
          # Update observed landmark
          observed_land_idx = land_idx_ls[ml_idx]
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.updateObservedLandmark(self.pose_cov)

      else:
        # Use the given correspondence
        observed_land_idx = correspondences[meas_idx]

        # Check if we have the landmark, otherwise create a new one
        if observed_land_idx in self.landmarks:
          # Update observed landmark
          observed_landmark = self.landmarks[observed_land_idx]
          observed_landmark.samplePose(self.pose_mean, self.pose_cov, meas, meas_cov)
          observed_landmark.updateObservedLandmark(self.pose_cov)
        else:
          # Initialize new landmark
          observed_landmark = LandmarkEKF(random=self.random)
          self.landmarks[observed_land_idx] = observed_landmark
          observed_landmark.updateNewLandmark(self.pose + self.computePoseNoise(), self.pose_mean, self.pose_cov, meas, meas_cov, self.new_land_threshold)
        ml_idx = observed_land_idx
      if self.can_change_landmark:
        # Update unobserved landmarks
        landmark_idxs_to_remove = []
        for landmark_idx, landmark in self.landmarks.items():
          # Don't update observed landmark
          if landmark_idx != observed_land_idx:
            keep = landmark.updateUnobservedLandmark(sensor_range_ls[meas_idx],sensor_bearing_ls[meas_idx])
            if not keep:
              landmark_idxs_to_remove.append(landmark_idx)

        # Remove unobserved landmarks whose log odds probability of existing fell below 0
        for idx in landmark_idxs_to_remove:
          self.landmarks.pop(idx)
    
      # Update particle properties
      self.weight *= observed_landmark.weight
      self.weight = max(self.weight, 1e-50)
      self.pose_mean = np.copy(observed_landmark.pose_mean)
      self.pose_cov = np.copy(observed_landmark.pose_cov)
      self.pose = np.copy(observed_landmark.sampled_pose)

  def computeLandmarkSpecificVals(self, landmark: EKF,  pose_mean, pose_cov, meas, meas_cov):
    est_meas = self.robot.computeMeasModel(pose_mean) # range_est, bearing_est
    landmark.meas_jac_pose, landmark.meas_jac_land = self.robot.computeMeasJacobians(pose_mean, landmark.mean)
    landmark.innovation = meas - est_meas

    landmark.inv_pose_cov = np.linalg.inv(pose_cov) 
    landmark.Q = meas_cov + landmark.meas_jac_land @ landmark.cov @ landmark.meas_jac_land.T
    landmark.inv_Q = np.linalg.inv(landmark.Q)
    
  def updatePoseDistribution(self, landmark, pose_mean, pose_cov):
    pose_cov = np.linalg.inv(landmark.meas_jac_pose.T @ landmark.inv_Q @ landmark.meas_jac_pose + landmark.inv_pose_cov)
    pose_mean = pose_cov @ landmark.meas_jac_pose.T @ landmark.inv_Q @ landmark.innovation + pose_mean
    sampled_pose = pose_mean + self._random.multivariate_normal(np.zeros(self.robot.pose_dimension), pose_cov)

    return sampled_pose

  def getLandmarkLikelihood(self, sampled_pose, meas, landmark):
    improved_meas = self.robot.computeMeasModel(sampled_pose) #range_improved, bearing_improved
    improved_innovation = meas - improved_meas 
    
    exponent = -.5*(improved_innovation).T @ landmark.inv_Q @ improved_innovation
    two_pi_inv_Q_sqrt = (np.linalg.det(2*np.pi* landmark.Q)) ** -0.5
    prob_data_association = two_pi_inv_Q_sqrt * np.exp(exponent)
    
    return prob_data_association

  def updateObservedLandmark(self,landmark, pose_cov, meas_cov):
    landmark.exist_log += self._landmark_constants.exist_log_inc

    K =  landmark.mean @ landmark.meas_jac_land.T @ landmark.inv_Q 
    I = np.eye(landmark.mean.shape[0])

    # update landmark
    landmark.mean = landmark.mean + K @ landmark.innovation
    landmark.cov = ( I - K @ landmark.meas_jac_land) @ landmark.cov
    
    # Compute particle weight
    L = landmark.meas_jac_pose @ pose_cov @ landmark.meas_jac_pose.T \
         + landmark.meas_jac_land @ landmark.cov @ landmark.meas_jac_land.T \
         + meas_cov 
    L_inv = np.linalg.inv(L)
    two_pi_L_inv_sqrt = (np.linalg.det(2*np.pi*L)) ** -0.5
    exponent = -.5* landmark.innovation.T @ L_inv @ landmark.innovation 
    weight = two_pi_L_inv_sqrt * np.exp(exponent)

    return weight
    
  def keepUnobservedLandmark(self, landmark, sampled_pose):
    # Update probability of the landmark existing based on whether it should have been measured
    meas = self.robot.computeMeasModel(sampled_pose)
    if self.robot.isLandmarkInRange(meas):
      landmark.land_exist_log -= self._landmark_constants.exist_log_dec

    # If the log odds probability falls below 0, we do not keep the landmark
    return landmark.land_exist_log >= 0

  def initLandmark(self, landmark, sampled_pose, meas_cov):
    # Initialize landmark mean and covariance, as well as previous mean and covariance
    landmark.mean = self.computeMeasModelInverse(sampled_pose)
    landmark.cov = np.linalg.inv(landmark.meas_jac_land @ np.linalg.inv(meas_cov) @ self.meas_jac_land.T)
    return landmark
