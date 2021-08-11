import copy
from slam.fast_slam2.FastSLAM2 import FastSLAM2
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D
import numpy as np
from slam.Utils import wrap_to_pi

class SimSensor():
    def __init__(self, name, update_period, limit = [2,np.pi/3], noise_std = [1, np.pi/36]): 
      # velocity, angular velocity, additional dimensions for limits and noise_std
      self.limit = np.array(limit)
      self.noise_std = noise_std
      self.name = name
      self.update_period = update_period

class RobotSimulator():
  def __init__(self, robot, sensors, landmarks,velocity , velocity_std, random, initial_pose, initial_pose_cov, verbose=False):
    self.velocity_std = velocity_std # all in meter and radian
    self.sensors = sensors
    self.dt = 1
    self.t = 0

    self.velocity = velocity

    self.robot = robot
    self.robot_pose = initial_pose
    self.robot_history = []
    self.landmarks = landmarks

    self._verbose = verbose
  def _log(self,*msg):
    if self._verbose:
      print('Robot Simulator:', *msg)

  def set_target_location(self, target_location: "tuple[float]"):
    self.target_location = target_location #x,y
  def increment_time(self):
    self.t += self.dt

  def move_robot_and_read_control(self): 
    self._log('move robot and read control')
    computed_control = self.robot.compute_control(self.robot_pose, self.target_location,self.velocity, self._verbose)
    
    actual_control = self.robot.compute_actual_control(computed_control, self.velocity, self.velocity_std)
    
    previous_robot_pose = self.robot_pose
    
    self.robot_history.append(copy.deepcopy(self.robot_pose))
    self.robot_pose = self.robot.compute_motion_model(self.robot_pose, actual_control, self.dt)

    self._log('computed control',computed_control,'actual control',actual_control)
    self._log('pose before',previous_robot_pose, 'after',self.robot_pose)
    return computed_control

  def read_measurement(self):
    true_measurements = {}
    actual_measurements = []
    for key, landmark_pose in list(self.landmarks.items()):
      true_measurement = self.robot.compute_meas_model(self.robot_pose,landmark_pose)
      true_measurements[key] = true_measurement

    for sensor in self.sensors:
      if self.t % sensor.update_period != 0: continue
      for key, measurement in list(true_measurements.items()):
        meas = self.robot.compute_actual_measurement(measurement,sensor.limit, sensor.noise_std)
        if meas[0]:
          actual_measurements.append({'key':key, 'meas':meas, 'noise':sensor.noise_std, 'limit':sensor.limit})
    return actual_measurements
