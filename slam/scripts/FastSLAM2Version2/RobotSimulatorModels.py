import copy
from FastSLAM2 import FastSLAM2
from RobotPhysics2D import RobotPhysics2D
import numpy as np
from Util import wrapToPi

class Sensor():
    def __init__(self, name, update_period, limits = {'range':1,'bearing':np.pi/4}, noise_std = {'range':1, 'bearing': np.pi/36}):
      self.limits = limits
      self.noise_std = noise_std
      self.name = name
      self.update_period = update_period
class RobotPhysics2DForSim(RobotPhysics2D):
  def __init__(self, random):
    super().__init__(random)

  def compute_control(self, robot_pose, target, velocity, angular_velocity):
    range_meas, bearing_meas = self.compute_meas_model(robot_pose, target)
    if abs(bearing_meas)<.1: 
      return velocity,0
    else:
      direction = 1 if 0<=bearing_meas <=np.pi else -1
      return 0,direction * angular_velocity

  def compute_actual_measurement(self,measurement, sensor_limits, sensor_noise_std):
    range_mea, bearing_mea = measurement
    if abs(bearing_mea)>sensor_limits['bearing'] or range_mea > sensor_limits['range']:
      return None, None
    return np.random.normal(range_mea,sensor_noise_std['range']),np.random.normal(bearing_mea,sensor_noise_std['bearing'])

  def is_close(self, robot_pose,target,acceptable_range, acceptable_bearing):
    measurement = compute_true_measurement(robot_pose,target)
    measurement = compute_actual_measurement(measurement,{'range':acceptable_range,'bearing':acceptable_bearing},{'range':0,'bearing':0})
    if measurement[0]: return True
    return False

class RobotSimulator():
  def __init__(self, sensors, landmarks,velocity, angular_velocity , robot_motion_std, random,initial_pose, default_pose_cov):
    self.robot_motion_std = robot_motion_std # all in meter and radian
    self.sensors = sensors
    self.dt = 1
    self.t = 0

    self.velocity, self.angular_velocity = velocity, angular_velocity 

    self.robot = RobotPhysics2DForSim(random,initial_pose, default_pose_cov)
    self.robot_pose = [0,0,0]
    self.robot_history = []
    self.landmarks = landmarks

  def set_target(self, target: "tuple[float]"):
    self.target = target #x,y
  def increment_time(self):
    self.t += self.dt

  def move_robot_and_read_control(self): 
    v, w = self.robot.compute_control(self.robot_pose, self.target,self.velocity, self.angular_velocity)
    w = wrapToPi(w)
    computed_control = (v,w) #theoretical input

    v += np.random.normal(0, self.robot_motion_std['v']) 
    w += np.random.normal(0, self.robot_motion_std['w']) 
    w = wrapToPi(w)
    actual_control = (v,w)

    self.robot_history.append(copy.deepcopy(self.robot_pose))
    self.robot_pose = self.robot.compute_motion_model(self.robot_pose, actual_control, self.dt)
    return computed_control
  
  def read_measurement(self):
    true_measurements = {}
    actual_measurents = []
    for key, landmark_pose in list(self.landmarks.items()):
      true_measurement = self.robot.compute_meas_model(self.robot_pose,landmark_pose)
      true_measurements[key] = true_measurement
    
    for sensor in self.sensors:
      if self.t % sensor.update_period != 0: continue
      for key, measurement in list(true_measurements.items()):
        range_mea, bearing_mea = self.robot.compute_actual_measurement(measurement,sensor.limits, sensor.noise_std)
        if range_mea:
          actual_measurents.append((key, range_mea, bearing_mea, sensor.noise_std['range'], sensor.noise_std['bearing'], \
            sensor.limits['range'], sensor.limits['bearing']))
    return actual_measurents
