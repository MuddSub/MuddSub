from abc import ABC, abstractmethod
class RobotBase(ABC):
    def __init__(self, pose_dimension):
        self.pose_dimension = pose_dimension

        @abstract

    @abstractmethod
    def computeMeasModel(self):
        pass
    
    @abstractmethod
    def computeMeasJacobians(self):
        pass
    
    @abstractmethod
    def computeMeasModelInverse(self):
        pass

    @abstractmethod
    def computeMotionModel(self):
        pass

    @abstractmethod
    def isLandmarkInRange(self):
        pass
