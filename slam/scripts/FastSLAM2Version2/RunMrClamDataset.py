import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning)
import pickle
from Dataloader import *
from FastSLAM2 import FastSLAM2
import numpy as np
from Util import wrapToPi
from Models import Meas, FastSLAM2Parameters, LandmarkConstants
from RobotPhysics2D import RobotPhysics2D
from Validation import plot_data

class RunMRCLAMDataset():
    def __init__(self, **kwargs):
        self.hardcode_compass = kwargs.get('hardcode_compass',True)
        self.hardcode_meas = kwargs.get('hardcode_meas', False)
        self.no_measurements = kwargs.get('no_measurements', False)
        self.plot_avg = kwargs.get('plot_avg', True)
        self.init_landmarks = kwargs.get('init_landmarks', False)
        self.known_correspondences = kwargs.get('known_correspondences', True)
        self.robot_id = kwargs.get('robot_id', 0)
        self.start_step = kwargs.get('start_step', 0)
        self.num_steps = kwargs.get('num_steps', 1000)
        self.meas_cov = kwargs.get('meas_cov', np.diag([1e-2, 1e-2]))
        self.sensor_range = kwargs.get('sensor_range', 100)
        self.sensor_fov = kwargs.get('sensor_fov', np.pi)

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

    def loadData(self, pkl='../../datasets/Jar/dataset1.pkl'):
        global algorithm
        self.dataloader = pickle.load(open(pkl,'rb'))
        self.robotData = self.dataloader.robots[self.robot_id]
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
        for i in range(self.num_steps):
            self.update = self.robotData.getNext()
            t = self.update[1][0]
            self.groundtruth_path_data.append([self.robotData.getXTruth(t),self.robotData.getYTruth(t)])
            if i==0:
                self.algorithm.prev_t = t
                self.algorithm._robot_physics.initial_pose[2] = wrapToPi(self.robotData.getCompass(t))
            if self.update[0] == "odometry":
                #theta_meas = wrapToPi(robotData.getCompass(t))
                self._addControl(t)
                # Hard coding poses
                if self.hardcode_compass:
                    for j in range(len(self.algorithm.particles)):
                        x = self.algorithm.particles[j].pose[0]
                        y = self.algorithm.particles[j].pose[1]
                        theta = self.robotData.getCompass(t)
                        self.algorithm.particles[j].pose = np.array([x, y, theta])
            else:
                print("step", i)
                self._addMeas()
            self.logData(i)
        
    def plot(self):
        landmarksGroundtruth = []
        for _, landmark in self.dataloader.map.landmarkDict.items():
            landmarksGroundtruth.append(np.array([landmark['X'], landmark['Y']]))
        landmarksGroundtruth = np.array(landmarksGroundtruth)
        print("num landmark: ground truth", len(landmarksGroundtruth), " what we got", len(self.algorithm.particles[0].landmarks))
        plot_data(self.num_particles, self.plot_data_list,self.groundtruth_path_data, landmarksGroundtruth)
            
    def logData(self, i):
        _, *slam_snapshot = self.algorithm.get_pose_and_landmarks_for_plot()
        self.plot_data_list.append([i, *slam_snapshot])

    def _addControl(self, t):
        odometry = self.update[1]

        # Use groundtruth to calculate odometry input
        time, velocity, angular_velocity = odometry
        
        # Update particle poses
        self.algorithm.add_control((velocity, angular_velocity), t)

    def _addMeas(self):
        measurement = self.update[1]
        time, subject, range_meas, bearing_meas = measurement
        
        # Update EKFs
        if not self.no_measurements:
            
            if subject > 5:
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
                print( "updated measurement", subject, "with position", [landmark_x, landmark_y])
                
                meas = Meas((range_meas, bearing_meas), self.meas_cov, (self.sensor_range, self.sensor_fov), subject)         
                self.algorithm.add_measurement(meas)

clamDataSet = RunMRCLAMDataset(init_landmarks=True)
clamDataSet.loadData()
clamDataSet.runFastSlam2()
clamDataSet.plot()