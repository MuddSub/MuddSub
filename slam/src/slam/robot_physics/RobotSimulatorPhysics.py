import copy
from slam.fast_slam2.FastSLAM2 import FastSLAM2
from slam.robot_physics.RobotPhysics2D import RobotPhysics2D
import numpy as np
from slam.Utils import wrap_to_pi

class RobotPhysics2DForSim(RobotPhysics2D):
  def __init__(self, random, position_is_close, verbose=False):
    super().__init__(random)
    self._verbose = verbose
    self.position_is_close = position_is_close

  def _log(self, *msg):
    if self._verbose:
      print('Sim Physics:', *msg)

  def compute_control(self, robot_pose, target, velocity,  verbose=False):
    velocity, angular_velocity = velocity
    range_meas, bearing_meas = self.compute_meas_model(robot_pose, target)
    bearing_meas = wrap_to_pi(bearing_meas)
    assert -1 * np.pi <= bearing_meas <= np.pi

    if abs(bearing_meas)<self.position_is_close[1]: # bearing
      displacement = velocity
      if verbose: self._log("range", range_meas, "bearing", bearing_meas, "displacement", displacement)
      return displacement, 0
    else:
      direction = 1 if 0<=bearing_meas <=np.pi else -1
      angular_displacement_mag = angular_velocity if abs(bearing_meas)>angular_velocity else abs(bearing_meas)
      angular_displacement = angular_displacement_mag * direction
      if verbose: self._log("range", range_meas, "bearing", bearing_meas, "anglar displacement", angular_displacement)
      return 0, angular_displacement

  def compute_actual_control(self, control, velocity, velocity_std):
    v, w = control
    w = wrap_to_pi(w)
    v_std, w_std = velocity_std
    v_std *= .05 if v==0 else abs(v/velocity[0])
    w_std *= .05 if w==0 else abs(w/velocity[1])
    v = np.random.normal(v, v_std)
    w = np.random.normal(w, w_std)
    w = wrap_to_pi(w)
    actual_control = (v, w)
    return actual_control

  def compute_actual_measurement(self, measurement, sensor_limits, sensor_noise_std):
    range_mea, bearing_mea = measurement
    if sensor_limits[1]/2<=bearing_mea%(np.pi*2)<=2*np.pi-sensor_limits[1]/2 or range_mea > sensor_limits[0]:
      return None, None
    return np.random.normal(range_mea, sensor_noise_std[0]), np.random.normal(bearing_mea, sensor_noise_std[1])

  def is_close(self, robot_pose, target, acceptable_measurements):
    measurement = self.compute_meas_model(robot_pose, target)
    measurement = self.compute_actual_measurement(measurement, acceptable_measurements, [0, 0])
    return not measurement[0] is None

  def modify_particile_pose(self, particles, ground_truth_pose, hardcode_pose, hardcode_compass):
    if hardcode_pose or hardcode_compass:
      for j in range(len(particles)):
          if hardcode_pose:
              x = ground_truth_pose[0]
              y = ground_truth_pose[1]
          else:
              x = particles[j].pose[0]
              y = particles[j].pose[1]
          theta = ground_truth_pose[2]
          particles[j].pose = np.array([x, y, theta])
    return particles
