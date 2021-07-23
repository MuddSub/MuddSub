from Alfie import Alfie2D

class SimAlfie2D(Alfie2D):
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