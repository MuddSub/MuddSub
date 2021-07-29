import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import pickle
from Dataloader import *
from FastSLAM2 import FastSLAM2
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
#import argparses
from Util import wrapToPi
from Models import Meas, FastSLAM2Parameters, LandmarkConstants
from RobotPhysics2D import RobotPhysics2D
from Validation import plot_data

ROBOT_ID = 0
START_STEP = 0
NUM_STEPS = 20000
MEAS_COV = np.diag([0.01, 0.01])
SENSOR_RANGE = 1
SENSOR_BEARING = np.pi
LANDMARK_NUM = None #14

'''
HARDCODE_COMPASS = True
HARDCODE_MEAS = False
NO_MEASUREMENTS = False
PLOT_AVG = True
INIT_LANDMARKS = False
KNOWN_CORRESPONDENCES = True
'''

'''
threshold: .3, meas cov: .1, motion cov: 1e-2
'''

class RunMRCLAMDataset():
    def __init__(self, **kwargs):        
        self.hardcode_compass = kwargs.get('hardcode_compass',True)
        self.hardcode_meas = kwargs.get('hardcode_meas', False)
        self.no_measurements = kwargs.get('no_measurements', False)
        self.plot_avg = kwargs.get('plot_avg', True)
        self.init_landmarks = kwargs.get('init_landmarks', False)
        self.known_correspondences = kwargs.get('known_correspondences', True)

        self.plot_data_list = []
        self.groundtruth_path_data = []
        self.num_particles = 2
        self.default_pose_cov = np.diag([1e-4,1e-4,1e-4])

        self.params = FastSLAM2Parameters(
            num_particles = self.num_particles,
            is_landmarks_fixed = True, 
            initial_landmarks = {},
            landmark_constants = LandmarkConstants()
        )

        self.random_generator = np.random.default_rng()
        self.algorithm = None
        self.update = None

    def loadData(self, pkl =  '../../datasets/Jar/dataset1.pkl'):
        global algorithm
        self.dataloader = pickle.load(open(pkl,'rb'))
        self.robotData = self.dataloader.robots[ROBOT_ID]
        if self.init_landmarks:
            self.params.initial_landmarks = {}
            for idx, landmark in self.dataloader.map.landmarkDict.items():
                x = landmark['X']
                y = landmark['Y']
                self.params.initial_landmarks[idx] = np.array([np.array([x, y]), None])

    def runFastSlam2(self):
        random = np.random.default_rng()
        initial_pose = np.array([self.robotData.getXTruth(0),self.robotData.getYTruth(0), self.robotData.getCompass(0)])
        robot_physics = RobotPhysics2D(random, initial_pose, self.default_pose_cov)
        self.algorithm = FastSLAM2(robot_physics, parameters = self.params, random = random)

        theta = 0
        for i in range(NUM_STEPS):
            self.update = self.robotData.getNext()
            t = self.update[1][0]
            self.groundtruth_path_data.append([self.robotData.getXTruth(t),self.robotData.getYTruth(t)])
            if i==0:
                self.algorithm.prev_t = t
                self.algorithm._robot_physics.initial_pose[2] = wrapToPi(self.robotData.getCompass(t))
            if self.update[0] == "odometry":
                #theta_meas = wrapToPi(robotData.getCompass(t))
                self.addControl(t)
                # Hard coding poses
                if self.hardcode_compass:
                    for j in range(len(self.algorithm.particles)):
                        x = self.algorithm.particles[j].pose[0]
                        y = self.algorithm.particles[j].pose[1]
                        theta = self.robotData.getCompass(t)
                        self.algorithm.particles[j].pose = np.array([x, y, theta])
            else:
                print("step", i)
                self.addMeas()
                
            self.logData(i)
        
        self.getGroundtruth()

    def getGroundtruth(self):
        landmarksGroundtruth = []
        for idx, landmark in self.dataloader.map.landmarkDict.items():
            if LANDMARK_NUM == None or idx == LANDMARK_NUM:
                landmarksGroundtruth.append(np.array([landmark['X'], landmark['Y']]))
        landmarksGroundtruth = np.array(landmarksGroundtruth)
        print("num landmark: ground truth", len(landmarksGroundtruth), " what we got", len(self.algorithm.particles[0].landmarks))
        plot_data(self.num_particles, self.plot_data_list,self.groundtruth_path_data, landmarksGroundtruth)
            
    def logData(self,i):
        # Log data
        _, *slam_snapshot = self.algorithm.get_pose_and_landmarks_for_plot()
        self.plot_data_list.append([i,*slam_snapshot])

    def addControl(self, t):
        odometry = self.update[1]
        # Use groundtruth to calculate odometry input
        time, velocity, angular_velocity = odometry
        # Update particle poses
        self.algorithm.add_control((velocity, angular_velocity), t)

    def addMeas(self):
        measurement = self.update[1]
        time, subject, range_meas, bearing_meas = measurement
        
        # Update EKFs
        if not self.no_measurements:
            
            if (LANDMARK_NUM == None and subject > 5) or subject == LANDMARK_NUM: #if subject > 5 :
                landmark = self.dataloader.map.getLandmarkLocation(subject)
                landmark_x = landmark['X']
                landmark_y = landmark['Y']

                # Use groundtruth to provide accurate measurement
                if self.hardcode_meas:
                    robot_x = self.robotData.getXTruth(time)
                    robot_y = self.robotData.getYTruth(time)
                    robot_angle = self.robotData.getCompass(time)
                    range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
                    bearing_meas = wrapToPi(np.arctan2(landmark_y - robot_y, landmark_x - robot_x) - robot_angle)
                #print("step", i, "updated measurement", subject, "with position", [landmark_x, landmark_y])
                print( "updated measurement", subject, "with position", [landmark_x, landmark_y])
                
                meas = Meas((range_meas, bearing_meas), MEAS_COV, (SENSOR_RANGE, SENSOR_BEARING), subject)         
                self.algorithm.add_measurement(meas)

clamDataSet = RunMRCLAMDataset()
clamDataSet.loadData()
clamDataSet.runFastSlam2()