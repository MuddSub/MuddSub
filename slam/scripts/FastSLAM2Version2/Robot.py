from abc import ABC, abstractmethod
from scripts.FastSLAM2Version2.RobotBase import RobotBase
class Robot(RobotBase):
    def __init__(self, pose_dimension):
        self.pose_dimension = pose_dimension

    def computeMeasModel(self):
        pass
    
    def computeMeasJacobians(self):
        pass
    
    def computeMeasModelInverse(self):
        pass

    def computeMotionModel(self):
        pass

    def isLandmarkInRange(self):
        pass
