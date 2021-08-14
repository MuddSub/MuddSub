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
import argparse
from Util import wrap_to_pi
from Models import Meas,FastSLAM2Parameters
from RobotPhysics2D import RobotPhysics2D
from Validation import plot

ROBOT_ID = 0
START_STEP = 0
NUM_STEPS = 20000
MEAS_COV = np.diag([0.01, 0.01])
SENSOR_RANGE = 1
SENSOR_BEARING = np.pi
LANDMARK_NUM = None #14

"""HARDCODE_COMPASS = True
HARDCODE_MEAS = False
NO_MEASUREMENTS = False
PLOT_AVG = True
INIT_LANDMARKS = False
KNOWN_CORRESPONDENCES = True"""

'''
threshold: .3, meas cov: .1, motion cov: 1e-2
'''

class RobotBase():
    def __init__(self, **kwargs):

        
        self.hardcode_compass = kwargs.get('hardcode_compass',True)
        self.hardcode_meas = kwargs.get('hardcode_meas', False)
        self.no_measurements = kwargs.get('no_measurements', False)
        self.plot_avg = kwargs.get('plot_avg', True)
        self.init_landmarks = kwargs.get('init_landmarks', False)
        self.known_correspondences = kwargs.get('known_correspondences', True)



        self.plot_data = []
        self.groundtruth_path_data = []
        self.num_particles = 2
        self.default_pose_cov = np.diag([1e-4,1e-4,1e-4])

        self.params = FastSLAM2Parameters(
            num_particles = self.num_particles,
            is_landmarks_fixed = True, 
            new_landmark_threshold = .1,
            initial_landmarks = {}
        )
        self.random_generator = np.random.default_rng()
        self.algorithm = None

    def load_data(self, pkl =  '../datasets/Jar/dataset1.pkl'):
        global algorithm
        self.dataloader = pickle.load(open(pkl,'rb'))
        self.robot_data = self.dataloader.robots[ROBOT_ID]
        if self.init_landmarks:
            self.params.initial_landmarks = {}
            for idx, landmark in self.dataloader.map.landmark_dict.items():
                x = landmark['X']
                y = landmark['Y']
                self.params.initial_landmarks[idx] = np.array([x, y])
        
        self.run_fast_slam2()

    def run_fast_slam2(self):
        
        random = np.random.default_rng()
        initial_pose = np.array([self.robot_data.get_x_truth(0),self.robot_data.get_y_truth(0), self.robot_data.get_compass(0)])
        robot_physics = RobotPhysics2D(random, initial_pose, self.default_pose_cov)
        self.algorithm = FastSLAM2(robot_physics, parameters = self.params, random = random)

        theta = 0
        for i in range(NUM_STEPS):
            self.update = self.robot_data.get_next() #TODO
            
            t = self.update[1][0]
            self.groundtruth_path_data.append([self.robot_data.get_x_truth(t),self.robot_data.get_y_truth(t)])
            if i==0:
                self.algorithm.prev_t = t
                self.algorithm._robot_physics.initial_pose[2] = wrap_to_pi(self.robot_data.get_compass(t))
            if self.update[0] == "odometry":
                #theta_meas = wrapToPi(robotData.getCompass(t))
                control = self.get_control()
                self.algorithm.add_control(control, t)

                # Hard coding poses
                if self.hardcode_compass:
                    for j in range(len(self.algorithm.particles)):
                        x = self.algorithm.particles[j].pose[0]
                        y = self.algorithm.particles[j].pose[1]
                        theta = self.robot_data.get_compass(t)
                        self.algorithm.particles[j].pose = np.array([x, y, theta])
            else:
                print("step", i)
                meas = self.get_meas()
                self.algorithm.add_measurement(meas)
                
            self.log_data(i)
        
        self.get_groundtruth()
            
    
    def get_groundtruth(self):
        landmarks_groundtruth = []
        for idx, landmark in self.dataloader.map.landmark_dict.items():
            if LANDMARK_NUM == None or idx == LANDMARK_NUM:
                landmarks_groundtruth.append(np.array([landmark['X'], landmark['Y']]))
        landmarks_groundtruth = np.array(landmarks_groundtruth)
        print("num landmark: ground truth", len(landmarks_groundtruth), " what we got", len(self.algorithm.particles[0].landmarks))
        plot(self.num_particles, self.plot_data,self.groundtruth_path_data, landmarks_groundtruth)
            
    def log_data(self,i):
        # Log data
        _, *slam_snapshot = self.algorithm.get_pose_and_landmarks_for_plot()
        self.plot_data.append([i,*slam_snapshot])

    def get_control(self):
        odometry = self.update[1]
        # Use groundtruth to calculate odometry input
        time, velocity, angular_velocity = odometry
        # print("step", i, "updated odometry", (vx, vy, theta_imu, omega_imu))

        # Update particle poses
        # algorithm.addControl((vx, vy, theta_imu, omega_imu), t)
        return (velocity, angular_velocity)
        
    def get_meas(self):
        measurement = self.update[1]
        time, subject, range_meas, bearing_meas = measurement
        if not self.no_measurements:
            # Update EKFs
            if (LANDMARK_NUM == None and subject > 5) or subject == LANDMARK_NUM: #if subject > 5 :
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
                #print("step", i, "updated measurement", subject, "with position", [landmark_x, landmark_y])
                print( "updated measurement", subject, "with position", [landmark_x, landmark_y])
                
                meas = Meas((range_meas, bearing_meas),MEAS_COV,(SENSOR_RANGE, SENSOR_BEARING), subject)         
                return meas



def error_ellipse(points, cov, nstd=2):
    """
    Source: http://stackoverflow.com/a/12321306/1391441
    """
    def eigsorted(cov):
      '''
      Eigenvalues and eigenvectors of the covariance matrix.
      '''
      vals, vecs = np.linalg.eigh(cov)
      order = vals.argsort()[::-1]
      return vals[order], vecs[:, order]

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[::-1, 0]))

    # Confidence level
    q = 2 * norm.cdf(nstd) - 1
    r2 = chi2.ppf(q, 2)

    width, height = 2 * np.sqrt(vals * r2)

    return width, height, theta
