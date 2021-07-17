from abc import ABC, abstractmethod
import numpy as np
from Util import wrapToPi
import RobotBase
class Alfie(RobotBase):
    def __init__(self, pose_dimension):
        self.pose_dimension = pose_dimension
    
    def computeMeasModel(self, pose, prev_land_mean):
        #x,y,theta, vx, vy, omega, theta_p = pose
        x, y, theta = pose
        lx,ly = prev_land_mean

        range_est = ((lx - x) ** 2 + (ly - y) ** 2) ** 0.5
        bearing_est = self.wrapToPi(np.arctan2((ly - y), (lx - x)) - theta) #TODO make imported

        return np.array([range_est, bearing_est])
    
    def computeMeasJacobians(self, pose, prev_land_mean):
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
        meas_jac_land = -1 * meas_jac_pose[:2, :2]
        
        return meas_jac_pose, meas_jac_land
        
        
    
    def computeMeasModelInverse(self,pose):
        #x, y, theta, vx, vy, omega, theta_p = pose
        x, y, theta = pose
        range_meas, bearing_meas = self.meas
        lx = x + range_meas * np.cos(theta + bearing_meas)
        ly = y + range_meas * np.sin(theta + bearing_meas)
        return np.array([lx, ly])
    
    def computeMotionModel(self, prev_pose, control, dt):
        v, w = control
        if -1e-10 <= w <= 1e-10:
            w = 1e-10

        x, y, theta = prev_pose
        next_theta = self.wrapToPi(theta + w * dt)
        next_x = x + v/w * (-np.sin(theta) + np.sin(next_theta))
        next_y = y + v/w * ( np.cos(theta) - np.cos(next_theta))
        return np.array([next_x, next_y, next_theta])

    
    def isLandmarkInRange(self, measurement, acceptable_range):
        return measurement < acceptable_range
