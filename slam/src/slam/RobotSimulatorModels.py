import copy
from slam.FastSLAM2 import FastSLAM2
from slam.RobotPhysics2D import RobotPhysics2D
import numpy as np
from slam.Util import wrap_to_pi

class Sensor():
    def __init__(self, name, update_period, limits = {'range':2,'bearing':np.pi/3}, noise_std = {'range':1, 'bearing': np.pi/36}):
      self.limits = limits
      self.noise_std = noise_std
      self.name = name
      self.update_period = update_period
class RobotPhysics2DForSim(RobotPhysics2D):
  def __init__(self, random,initial_pose, default_pose_cov, bearing_is_close,verbose = False,):
    self._verbose = verbose
    super().__init__(random,initial_pose, default_pose_cov)
    self.bearing_is_close = bearing_is_close

  def _log(self,*msg):
    if self._verbose:
      print('Sim Physics:', *msg)
    
  def compute_control(self, robot_pose, target, velocity, angular_velocity, verbose=False):
    range_meas, bearing_meas = self.compute_meas_model(robot_pose, target)
    bearing_meas = wrap_to_pi(bearing_meas)
    assert -1*np.pi<=bearing_meas<=np.pi
    if abs(bearing_meas)<self.bearing_is_close: 
      displacement = velocity
      if verbose: self._log("range",range_meas,"bearing",bearing_meas, "displacement",displacement)
      return displacement,0
    else:
      direction = 1 if 0<=bearing_meas <=np.pi else -1
      angular_displacement_mag = angular_velocity if abs(bearing_meas)>angular_velocity else abs(bearing_meas)
      angular_displacement = angular_displacement_mag * direction
      if verbose: self._log("range",range_meas,"bearing",bearing_meas,"anglar displacement",angular_displacement)
      return 0,angular_displacement

  def compute_actual_measurement(self,measurement, sensor_limits, sensor_noise_std):
    range_mea, bearing_mea = measurement
    if sensor_limits['bearing']/2<=bearing_mea%(np.pi*2)<=2*np.pi-sensor_limits['bearing']/2 or range_mea > sensor_limits['range']:
      return None, None
    return np.random.normal(range_mea,sensor_noise_std['range']),np.random.normal(bearing_mea,sensor_noise_std['bearing'])

  def is_close(self, robot_pose,target,acceptable_range, acceptable_bearing):
    measurement = self.compute_meas_model(robot_pose,target)
    measurement = self.compute_actual_measurement(measurement,{'range':acceptable_range,'bearing':acceptable_bearing},{'range':0,'bearing':0})
    return not measurement[0] is None

class RobotSimulator():
  def __init__(self, sensors, landmarks,velocity, angular_velocity , robot_motion_std, random,initial_pose, default_pose_cov, bearing_is_close, verbose=False):
    self.robot_motion_std = robot_motion_std # all in meter and radian
    self.sensors = sensors
    self.dt = 1
    self.t = 0

    self.velocity, self.angular_velocity = velocity, angular_velocity 

    self.robot = RobotPhysics2DForSim(random,initial_pose, default_pose_cov, bearing_is_close, verbose = verbose)
    self.robot_pose = [0,0,0]
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
    v, w = self.robot.compute_control(self.robot_pose, self.target_location,self.velocity, self.angular_velocity, self._verbose)
    w = wrap_to_pi(w)
    computed_control = (v,w) #theoretical input

    v_std = self.robot_motion_std['v'] 
    w_std = self.robot_motion_std['w']     
    v_std *= .05 if v==0 else abs(v/self.velocity)
    w_std *= .05 if w==0 else abs(w/self.angular_velocity)
    v = np.random.normal(v, v_std)
    w = np.random.normal(w, w_std)
    w = wrap_to_pi(w)
    actual_control = (v,w)

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
        range_mea, bearing_mea = self.robot.compute_actual_measurement(measurement,sensor.limits, sensor.noise_std)
        if range_mea:
          actual_measurements.append((key, range_mea, bearing_mea, sensor.noise_std['range'], sensor.noise_std['bearing'], \
            sensor.limits['range'], sensor.limits['bearing']))
    return actual_measurements
