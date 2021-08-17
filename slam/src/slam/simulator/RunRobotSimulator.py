import copy
from slam.fast_slam2.FastSLAM2 import FastSLAM2
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D
from slam.robot_physics.RobotSimulatorPhysics import RobotPhysics2DForSim
import numpy as np
import pandas as pd
from slam.Utils import wrap_to_pi, plot_df
from slam.simulator.RobotSimulatorModels import RobotSimulator, SimSensor
from slam.fast_slam2.Models import Meas, FastSLAM2Parameters, LandmarkConstants
from abc import ABC, abstractmethod

class RobotSimulatorRunner():
  '''
  This is a runner for robot simulator. Robot simulator simulates the interaction between FastSLAM, 
  dummy perception, and dummy control (it uses very simple physics plus noise). 
  The robot takes in a target, and moves toward the target.
  It will perform either translation or rotation at every time step in order to approach this target. 
  It will only perform translation if it is close enough in terms of angular displacement.   
  This class is FastSLAM version agonistic and physics-agonistic. 
  '''
  def __init__(self, robot_physics, random, 
                num_steps, hardcode_compass, hardcode_pose, 
                close_enough_meas_to_update_target, close_enough_position_for_motion,
                sensors, velocity, velocity_std,
                landmarks, landmark_initial_noises, landmark_convs, 
                initial_pose, initial_pose_cov, num_particles, new_landmark_threshold,
                verbose = True,
                fast_slam_version=2, plot_msg = 'Fast SLAM 2'):
    self.num_steps = num_steps
    self.hardcode_compass = hardcode_compass
    self.hardcode_pose = hardcode_pose
    self.close_enough_meas_to_update_target = close_enough_meas_to_update_target
    self.close_enough_position_for_motion = close_enough_position_for_motion

    landmarks = {str(idx):landmark for idx, landmark in enumerate(landmarks)}

    estimated_init_landmarks = {key:
      (
        np.array([ random.normal(x, x_noise) for x, x_noise in zip(landmark_pos, landmark_initial_noises) ]),
        np.diag(landmark_convs)
      )
      for key, landmark_pos in list(landmarks.items())}
    self._log("Estimated init landmark", estimated_init_landmarks)

    self.num_particles = num_particles
    landmark_constants = LandmarkConstants()
    landmark_constants.new_landmark_threshold = new_landmark_threshold
    self.params = FastSLAM2Parameters(
      num_particles = self.num_particles,
      are_landmarks_fixed = True,
      initial_landmarks = estimated_init_landmarks,
      initial_pose = initial_pose,
      initial_pose_cov = initial_pose_cov,
      landmark_constants = landmark_constants,
      verbose = 2 if verbose else 0, 
      fast_slam_version = fast_slam_version,
    )
    self.sim = RobotSimulator(robot_physics, sensors, landmarks, velocity, velocity_std, random, initial_pose, initial_pose_cov, self.close_enough_position_for_motion)
    self.slam = FastSLAM2(robot_physics, parameters = self.params, random = random)

    self.meas_hist = []
    self.plot_data = pd.DataFrame()

    self.plot_msg = plot_msg

  def initialize(self):
    self.curr_target = str(0)
    self.final_target = str(len(self.sim.landmarks)-1)
    self.sim.set_target_location(self.sim.landmarks[self.curr_target])

    self.slam_robot_pose = None
    self.landmark_maps = None

  def _log(self, *msg):
    print('Simulator: ', *msg)

  def update_slam_snapshot(self, i):
    self._log('Step', i)

    # plotting, and also get current slam state
    snapshot = self.slam.get_pose_and_landmarks_for_plot()
    self.landmark_maps = self.slam.get_landmark_map()

    self.slam_robot_pose = snapshot['particle_poses'][0][snapshot['best_particle_idx'][0]]

    self._log('Slam robot pose', self.slam_robot_pose, "actual robot pose", self.sim.robot_pose)
    self._log('Target', self.curr_target, 'slam map location', self.sim.target_location, 'actual location', self.sim.landmarks[self.curr_target])

    self.plot_data = pd.concat([self.plot_data, snapshot], ignore_index=True)
    self.sim.set_target_location(self.landmark_maps[self.curr_target])

  def is_terminated(self):
    return self.sim.robot.is_close(self.slam_robot_pose, self.landmark_maps[self.final_target], self.close_enough_meas_to_update_target)

  def update_target(self):
    # move on to new target
    if self.sim.robot.is_close(self.slam_robot_pose, self.landmark_maps[self.curr_target], self.close_enough_meas_to_update_target):
      self.curr_target = str(int(self.curr_target)+1)

    self.sim.set_target_location(self.landmark_maps[self.curr_target])

  def update_meas(self):
    actual_measurements = self.sim.read_measurement()
    self.meas_hist.append(len(actual_measurements))
    for measurement in actual_measurements:
      key = measurement['key']
      meas = measurement['meas']
      noise = measurement['noise']
      limit = measurement['limit']
      meas_cov = np.diag(noise)
      self.slam.add_measurement(Meas(meas, meas_cov, limit, key))

  def update_control(self):
    actual_control = self.sim.move_robot_and_read_control()
    self.slam.add_control(actual_control, self.sim.t)

    self.slam.particles = self.sim.robot.modify_particile_pose(self.slam.particles,  self.sim.robot_history[self.sim.t],
                            self.hardcode_pose , self.hardcode_compass)
    self.sim.increment_time()

  def plot(self):
    self.plot_data['num_measurements'] = self.meas_hist
    landmark_names =  [idx for idx, _ in list(self.sim.landmarks.items())]
    landmarks_ground_truth = np.array([np.array(pos) for idx, pos in list(self.sim.landmarks.items())])
    self._log("end of sim", "actual landmarks", self.sim.landmarks, "slam robot pose", self.slam_robot_pose, "actual robot pose", self.sim.robot_pose)
    plot_df(self.plot_data, self.sim.robot_history, landmarks_ground_truth, msg = self.plot_msg)

  def run(self):
    '''
    This is the main function with a control loop that calls all of the other functions.
    '''
    self.initialize()
    for i in range(self.num_steps):
      self.update_slam_snapshot(i)
      self.update_meas()
      self.update_control()
      if self.is_terminated():
        break
      self.update_target()

    self.plot()

if __name__ == '__main__':
  num_steps = 1000
  hardcode_compass = True
  hardcode_pose = False
  close_enough_meas_to_update_target = [.3, np.pi]
  close_enough_position_for_motion = [None, np.pi/36] # constrain when to translate based on angular displacement

  sensors = [SimSensor('sensor0', 1, limit = [2, np.pi/2], noise_std = [.3**.5, np.pi/36])] # range and bearing
  landmarks = [(1, 0), (1, 2), (3, 3), (4, 3),
                    (5, 5), (6, 7), (7, 8), (8, 9)]

  velocity  = np.array([.3, np.pi/4]) # linear velocity(technically not correct but informative), angular velocity
  velocity_std = velocity*np.array([.1, .1]) # did not use scalar we might want to scale each dimension differently

  landmark_initial_noises = [.5, .5]
  landmark_convs = [1.2, 1.2]

  initial_pose = np.array([0, 0, 0])
  initial_pose_cov = np.diag([.5**2, .5**2, (np.pi/360)**2])

  num_particles = 30
  new_landmark_threshold = 1.1

  random = np.random.default_rng()
  robot_physics = RobotPhysics2DForSim(random, close_enough_position_for_motion, verbose = True)

  runner = RobotSimulatorRunner(robot_physics, random, num_steps, hardcode_compass, hardcode_pose, close_enough_meas_to_update_target, close_enough_position_for_motion,
                  sensors, velocity, velocity_std,
                  landmarks, landmark_initial_noises, landmark_convs, initial_pose, initial_pose_cov, num_particles, new_landmark_threshold, verbose = True)
  runner.run()
