import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning)
import pickle
import pandas as pd
from Dataloader import *
from FastSLAM2 import FastSLAM2
import numpy as np
from Util import wrap_to_pi
from Models import Meas, FastSLAM2Parameters, LandmarkConstants
from RobotPhysics2D import RobotPhysics2D
from Validation import plot_data

class RunMRCLAMDataset():
    def __init__(self, **kwargs):
        self.hardcode_pose = kwargs.get('hardcode_pose', False)
        self.hardcode_compass = kwargs.get('hardcode_compass', False)
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

        self.history = None
        self.groundtruth_path_data = []
        self.num_particles = 2
        self.default_pose_cov = np.diag([1e-4,1e-4,1e-4])

        self.params = FastSLAM2Parameters(
            num_particles = self.num_particles,
            are_landmarks_fixed = True, 
            initial_landmarks = {},
            landmark_constants = LandmarkConstants(),
            localization_only = False
        )

        self.random_generator = np.random.default_rng()
        self.algorithm = None
        self.update = None

    def load_data(self, pkl='../../datasets/Jar/dataset1.pkl'):
        global algorithm
        self.dataloader = pickle.load(open(pkl,'rb'))
        self.robot_data = self.dataloader.robots[self.robot_id]
        if self.init_landmarks:
            self.params.initial_landmarks = {}
            for idx, landmark in self.dataloader.map.landmark_dict.items():
                x = landmark['X']
                y = landmark['Y']
                self.params.initial_landmarks[idx] = (np.array([x, y]), None)

    def run_fast_slam2(self):
        random = np.random.default_rng()
        initial_pose = np.array([self.robot_data.get_x_truth(0),self.robot_data.get_y_truth(0), self.robot_data.get_compass(0)])
        robot_physics = RobotPhysics2D(random, initial_pose, self.default_pose_cov)
        self.algorithm = FastSLAM2(robot_physics, parameters = self.params, random = random)

        theta = 0
        for i in range(self.num_steps):
            self.update = self.robot_data.get_next()
            t = self.update[1][0]
            self.groundtruth_path_data.append([self.robot_data.get_x_truth(t),self.robot_data.get_y_truth(t)])
            if i == 0:
                self.algorithm.prev_t = t
                self.algorithm._robot_physics.initial_pose[2] = wrap_to_pi(self.robot_data.get_compass(t))
            if self.update[0] == "odometry":
                #theta_meas = wrapToPi(robotData.getCompass(t))
                self._add_control(t)
                # Hard coding compass or pose
                if self.hardcode_pose or self.hardcode_compass:
                    for j in range(len(self.algorithm.particles)):
                        if self.hardcode_pose:
                            x = self.robot_data.get_x_truth(t)
                            y = self.robot_data.get_y_truth(t)
                        else:
                            x = self.algorithm.particles[j].pose[0]
                            y = self.algorithm.particles[j].pose[1]
                        theta = self.robot_data.get_compass(t)
                        self.algorithm.particles[j].pose = np.array([x, y, theta])
            else:
                print("step", i)
                self._add_meas()
            self.log_data(i)
        
    def plot(self):
        landmarks_groundtruth = []
        for _, landmark in self.dataloader.map.landmark_dict.items():
            landmarks_groundtruth.append(np.array([landmark['X'], landmark['Y']]))
        landmarks_groundtruth = np.array(landmarks_groundtruth)
        print("num landmark: ground truth", len(landmarks_groundtruth), " what we got", len(self.algorithm.particles[0].landmarks))
        plot_data(self.num_particles, self.history, self.groundtruth_path_data, landmarks_groundtruth)
            
    def log_data(self, i):
        snapshot = self.algorithm.get_pose_and_landmarks()
        if self.history == None:
            self.history = snapshot
        else:
            self.history = pd.concat([self.history, snapshot], ignore_index=True)

    def _add_control(self, t):
        odometry = self.update[1]

        # Use groundtruth to calculate odometry input
        time, velocity, angular_velocity = odometry
        
        # Update particle poses
        self.algorithm.add_control((velocity, angular_velocity), t)

    def _add_meas(self):
        measurement = self.update[1]
        time, subject, range_meas, bearing_meas = measurement
        
        # Update EKFs
        if not self.no_measurements:
            
            if subject > 5:
                landmark = self.dataloader.map.get_landmark_location(subject)
                landmark_x = landmark['X']
                landmark_y = landmark['Y']

                # Use groundtruth to provide accurate measurement
                if self.hardcode_meas:
                    robot_x = self.robot_data.get_x_truth(time)
                    robot_y = self.robot_data.get_y_truth(time)
                    robot_angle = self.robot_data.get_compass(time)
                    range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
                    bearing_meas = wrap_to_pi(np.arctan2(landmark_y - robot_y, landmark_x - robot_x) - robot_angle)
                print( "updated measurement", subject, "with position", [landmark_x, landmark_y])
                
                meas = Meas((range_meas, bearing_meas), self.meas_cov, (self.sensor_range, self.sensor_fov), subject)         
                self.algorithm.add_measurement(meas)

clam_dataset = RunMRCLAMDataset(init_landmarks=True)
clam_dataset.load_data()
clam_dataset.run_fast_slam2()
clam_dataset.plot()