from abc import ABC, abstractmethod
import numpy as np
from Util import wrapToPi
from RobotPhysicsBase import RobotPhysicsBase

class RobotPhysics2D(RobotPhysicsBase):
    def __init__(self, random, initial_pose, default_pose_cov = None):
        super().__init__(random)
        self.initial_pose = initial_pose
        self.default_pose_cov = default_pose_cov

    def update_default_pose_cov(self, x_sigma, y_sigma, theta_sigma):
        self.default_pose_cov = np.array([x_sigma, y_sigma, theta_sigma])

    def compute_meas_model(self, pose, landmark_mean):
        #x,y,theta, vx, vy, omega, theta_p = pose
        x, y, theta = pose
        lx,ly = landmark_mean

        range_est = ((lx - x) ** 2 + (ly - y) ** 2) ** 0.5
        bearing_est = wrapToPi(np.arctan2((ly - y), (lx - x)) - theta) #TODO make imported

        return np.array([range_est, bearing_est])
    
    def compute_meas_jacobians(self, pose, landmark_mean):
        lx, ly = landmark_mean
        x, y, theta, = pose
        
        range_est_sqr = ((lx-x)**2+(ly-y)**2)
        range_est = range_est_sqr**.5

        meas_jac_pose = np.zeros((2,3))
        
        meas_jac_pose[0, 0] = -(lx-x) / range_est
        meas_jac_pose[0, 1] = -(ly-y) / range_est
        meas_jac_pose[1, 0] = (ly-y) / range_est_sqr
        meas_jac_pose[1, 1] = -(lx-x) / range_est_sqr
        meas_jac_pose[1, 2] = -1
        meas_jac_land = -1 * meas_jac_pose[:2, :2]
        
        return meas_jac_pose, meas_jac_land
        
    def compute_inverse_meas_model(self,pose,meas):
        #x, y, theta, vx, vy, omega, theta_p = pose
        x, y, theta = pose
        range_meas, bearing_meas = meas
        lx = x + range_meas * np.cos(theta + bearing_meas)
        ly = y + range_meas * np.sin(theta + bearing_meas)
        return np.array([lx, ly])
    
    def compute_motion_model(self, pose, control, dt):
        v, w = control
        if -1e-10 <= w <= 1e-10:
            w = 1e-10

        x, y, theta = pose
        next_theta = wrapToPi(theta + w * dt)
        next_x = x + v/w * (-np.sin(theta) + np.sin(next_theta))
        next_y = y + v/w * ( np.cos(theta) - np.cos(next_theta))
        return np.array([next_x, next_y, next_theta])
    
    def 
    