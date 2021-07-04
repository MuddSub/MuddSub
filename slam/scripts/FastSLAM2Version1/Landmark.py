import numpy as np
from scipy.linalg import sqrtm
# to do: turn landmark into matrices, and particles into matrices 
# aka numpify (but not now)

'''
s: pose --> robot pose (3 states)
sigma_s_t,n_t: pose_cov_expected --> the pose cov for the **new** pose sampling distribution in data association.  
sigma_s_t,n_t: pose_mean_expected --> the pose mean for the **new** pose sampling distribution in data association. 
s_nt,t: sampled pose --> the pose sampled from the **new** distribution in data association

mu: land_mean --> landmark posiion mean (x,y) 
sigma: land_cov --> landmark position covariance

mu_t-1: prev_land_mean --> landmark position mean from previous time step
sigma_t-1: prev_land_cov --> landmark position covariance from previous time step

z: meas = (meas_range, meas_bearing) --> measurement
z_hat: meas_est = (meas_range_est, meas_bearing_est) --> predicted measurement
z - z_hat:  meas_diff

Gs: meas_jac_pose --> Jacobian of the measurement model with respect to the robot pose
Gtheta: meas_jac_land --> Jacobian of the measurement model with respect to the landmark position

g: measurement_model
R: meas_cov: measurement model noise covariance

h: motion_model (on pose)
P: pose_cov: motion model noise covariance

tau: land_exist_log --> log odds of the probability that the landmark exists
rho+: land_exist_log_inc --> log odds value to add to land_exist_log when positive evidence of the landmark is seen
rho-: land_exist_log_dec --> log odds value to subtract to land_exist_log when negative evidence of the landmark is seen

Q: Q --> fused measurement covariance. meas_cov + meas_jac_land @ land_cov @ meas_jac_land
    (2,2)
Q: Q_inv --> Q inverse
K: K --> Kalman gain (2,2)
I: identity matrix with the shape of KGtheta, !!!!!! Problem: size mismatched

w: weight 

p_nt: prob_match --> probability of landmark associates with given measurement
'''
def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi
  if th <= -np.pi:
      th += 2*np.pi
  return th

class LandmarkEKF():
  def __init__(self, land_mean=np.zeros(2), land_cov=np.eye(2), random=None):
    self.prev_land_mean = land_mean
    self.prev_land_cov = land_cov

    self.land_mean = np.copy(land_mean)
    self.land_cov = np.copy(land_cov)

    # In FastSLAM 2.0, the pose is sampled taking into account the measurement and the data association variable. 
    # If the data association variable is not known, the sampled pose is used to calculate the probability of having
    # observed the landmark given the measurement, and the landmark that maximizes this probability is chosen. That
    # is why the sampled pose is saved in the EKF.
    self.sampled_pose = np.zeros(3)

    self.meas = np.zeros(2)
    self.meas_diff = np.zeros(2)

    self.meas_jac_pose, self.meas_jac_land = np.zeros((2,3)),np.zeros((2,3))

    # self.meas_cov = np.zeros((2,2))
    # self.pose_cov = np.zeros((3,3))
    # self.pose_cov_inv = self.pose_cov

    self.Q = np.zeros((2,2))
    self.Q_inv = np.zeros((2,2)) 
    #self.K = np.zeros((2,2))
    self.I = np.eye(2) #!!!!!! Problem: size mismatched

    self.random = random
    self.prob_data_association = 0
    
    self.land_exist_log = 0
    self.land_exist_log_threshold = self.logOdds(0.5)
    self.land_exist_log_inc = self.logOdds(0.8) # 0.8 works out to around 1.38, rho positive in the FastSLAM 2.0 algorithm
    self.land_exist_log_dec = self.logOdds(0.8) # 0.8 works out to around 1.38, rho negative in the FastSLAM 2.0 algorithm

    self.label = []
  
  def logOdds(self, probability):
    return np.log(probability / (1 - probability))

  def gauss(self, x, mu, std):
    a = 1 / (std * np.sqrt(2 * np.pi))
    b = -0.5 / (std ** 2)
    g = a * np.exp(b * (x - mu) ** 2)
    return g

  def computeMeasModel(self, pose):
    #x,y,theta, vx, vy, omega, theta_p = pose
    x, y, theta = pose
    lx, ly = self.prev_land_mean

    range_est = ((lx - x) ** 2 + (ly - y) ** 2) ** 0.5
    bearing_est = wrapToPi(np.arctan2((ly - y), (lx - x)) - theta)

    return np.array([range_est, bearing_est])

  def computeMeasModelInverse(self, pose):
    #x, y, theta, vx, vy, omega, theta_p = pose
    x, y, theta = pose
    range_meas, bearing_meas = self.meas
    lx = x + range_meas * np.cos(theta + bearing_meas)
    ly = y + range_meas * np.sin(theta + bearing_meas)
    return np.array([lx, ly])

  def computeMeasJacobians(self, pose):
    lx, ly = self.prev_land_mean
    x, y, theta, = pose
    
    range_est_sqr = ((lx-x)**2+(ly-y)**2)
    range_est = range_est_sqr**.5

    meas_jac_pose = np.zeros((2,3))
    
    meas_jac_pose[0, 0] = -(lx-x) / range_est
    meas_jac_pose[0, 1] = -(ly-y) / range_est
    meas_jac_pose[1, 0] = (ly-y) / range_est_sqr
    meas_jac_pose[1, 1] = -(lx-x) / range_est_sqr
    meas_jac_pose[1, 2] = -1
    self.meas_jac_pose = meas_jac_pose
    self.meas_jac_land = -1 * meas_jac_pose[:2, :2]
  
  def samplePose(self, pose_mean, pose_cov, meas, meas_cov):
    # range_meas, bearing_meas = meas 
    self.meas = meas
    self.meas_cov = meas_cov
    meas_est = self.computeMeasModel(pose_mean) # range_est, bearing_est
    
    self.meas_diff = meas - meas_est #np.array([range_meas - range_est, bearing_meas - bearing_est])

    pose_cov_inv = np.linalg.inv(pose_cov)

    self.computeMeasJacobians(pose_mean)
    
    self.Q = self.meas_cov + self.meas_jac_land @ self.prev_land_cov @ self.meas_jac_land.T
    self.Q_inv = np.linalg.inv(self.Q)
    
    self.pose_cov = np.linalg.inv(self.meas_jac_pose.T @ self.Q_inv @ self.meas_jac_pose + pose_cov_inv)
    self.pose_mean = self.pose_cov @ self.meas_jac_pose.T @ self.Q_inv @ self.meas_diff + pose_mean
    self.sampled_pose = self.pose_mean + self.random.multivariate_normal(np.zeros(3), self.pose_cov)
    meas_improved = self.computeMeasModel(self.sampled_pose) #range_improved, bearing_improved
    improved_diff = meas - meas_improved 
    exponent = -.5*(improved_diff).T @ self.Q_inv @ (improved_diff)
    
    # two_pi_Q_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*self.Q)))
    two_pi_Q_inv_sqrt = (np.linalg.det(2*np.pi*self.Q)) ** -0.5
    
    self.prob_data_association = two_pi_Q_inv_sqrt * np.exp(exponent)
    # I think this should be a multiplication. but np.exp does return a matrix 
    # print('hiiii',self.prob_data_association, two_pi_Q_inv_sqrt, exponent, np.exp(exponent))
    return self.prob_data_association

  def computeWeightLocalization(self, pose, meas, meas_cov):
    range_meas, bearing_meas = meas
    range_meas_est, bearing_meas_est = self.computeMeasModel(pose)
    range_std = np.sqrt(meas_cov[0, 0])
    bearing_std = np.sqrt(meas_cov[1, 1])
    range_prob = self.gauss(range_meas_est, range_meas, range_std)
    bearing_prob = self.gauss(bearing_meas_est, bearing_meas, bearing_std)
    return range_prob * bearing_prob

  def updateObservedLandmark(self, pose_cov):
    self.land_exist_log += self.land_exist_log_inc
    K =  self.prev_land_cov @ self.meas_jac_land.T @ self.Q_inv 

    # Compute new landmark mean and covariance
    self.land_mean = self.prev_land_mean + K @ self.meas_diff
    self.land_cov = (self.I - K @ self.meas_jac_land) @ self.prev_land_cov #!!!!!! Problem: size mismatched
    
    # Compute particle weight
    L = self.meas_jac_pose @ pose_cov @ self.meas_jac_pose.T \
         + self.meas_jac_land @ self.prev_land_cov @ self.meas_jac_land.T \
         + self.meas_cov 
    L_inv = np.linalg.inv(L)
    # two_pi_L_inv_sqrt = np.linalg.inv(sqrtm(np.abs(2*np.pi*L)))
    two_pi_L_inv_sqrt = (np.linalg.det(2*np.pi*L)) ** -0.5
    exponent = -.5* self.meas_diff.T @ L_inv @ (self.meas_diff )
    weight = two_pi_L_inv_sqrt * np.exp(exponent)

    # Update previous values
    self.prev_land_mean = self.land_mean
    self.prev_land_cov = self.land_cov
    self.weight = weight
    #print("weight",weight)
    return weight
  # sensor_bearing considers the angle between the left most to the right most field of views
  def updateUnobservedLandmark(self, sensor_range, sensor_bearing = np.pi):
    # Set current landmark mean and cov to previous mean and cov
    self.land_mean = self.prev_land_mean
    self.land_cov = self.prev_land_cov

    # Update probability of the landmark existing based on whether it should have been measured
    range_meas, bearing_meas = self.computeMeasModel(self.sampled_pose)
    if range_meas <= sensor_range and bearing_meas <= sensor_bearing/2:
      self.land_exist_log -= self.land_exist_log_dec

    # If the log odds probability falls below 0, we do not keep the landmark
    return self.land_exist_log >= 0

  def updateNewLandmark(self, sampled_pose, pose_mean, pose_cov, meas, meas_cov, new_land_threshold):
    self.land_exist_log = self.land_exist_log_inc
    self.sampled_pose = sampled_pose
    self.pose_mean = pose_mean
    self.pose_cov = pose_cov
    self.meas = meas

    # Initialize landmark mean and covariance, as well as previous mean and covariance
    self.land_mean = self.computeMeasModelInverse(sampled_pose)
    self.prev_land_mean = self.land_mean
    self.computeMeasJacobians(sampled_pose)
    self.land_cov = np.linalg.inv(self.meas_jac_land @ np.linalg.inv(meas_cov) @ self.meas_jac_land.T)
    self.prev_land_cov = self.land_cov
    self.weight = new_land_threshold
