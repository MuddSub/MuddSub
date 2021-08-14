import copy
import numpy as np
from slam.robot_physics.RobotSimulatorPhysics import RobotPhysics2DForSim
from slam.simulator.RunRobotSimulator import *
if __name__ == '__main__':
  num_steps = 1000
  hardcode_compass = True
  hardcode_pose = False
  close_enough_meas_to_update_target = [.3, np.pi]
  close_enough_position_for_motion = [None, np.pi/36] # constrain when to translate based on angular displacement

  sensors = [Sensor('sensor0', 1, limits = [2, np.pi/2], noise_std = [.3**.5, np.pi/36])] # range and bearing
  landmarks = [(1, 0), (1, 2), (3, 3), (4, 3),
                    (5, 5), (6, 7), (7, 8), (8, 9)]

  velocity  = np.array([.3, np.pi/4]) # linear velocity(technically not correct but informative), angular velocity
  velocity_std = velocity*np.array([.1, .1]) # did not use scalar we might want to scale each dimension differently

  landmark_initial_noises = [.5, .5]
  landmark_convs = [1.2, 1.2]

  initial_pose = np.array([0, 0, 0])
  initial_pose_cov = np.diag([.5**2, .5**2, (np.pi/360)**2])

  num_particles = 1
  new_landmark_threshold = 1.1

  random = np.random.default_rng()
  robot_physics = RobotPhysics2DForSim(random, close_enough_position_for_motion, verbose = True)

  runner = RobotSimulatorRunner(robot_physics, random, num_steps, hardcode_compass, hardcode_pose, 
                  close_enough_meas_to_update_target, close_enough_position_for_motion,
                  sensors, velocity, velocity_std,
                  landmarks, landmark_initial_noises, landmark_convs, initial_pose, 
                  initial_pose_cov, 
                  num_particles, new_landmark_threshold, verbose = True,
                  fast_slam_version = 1, plot_msg = 'Fast SLAM 1')
  runner.run()
