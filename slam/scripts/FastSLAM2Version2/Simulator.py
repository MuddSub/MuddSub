import copy
from FastSLAM2 import FastSLAM2
import numpy as np
import plotter

def wrapToPi(th):
  th = np.fmod(th, 2*np.pi)
  if th >= np.pi:
      th -= 2*np.pi
  if th <= -np.pi:
      th += 2*np.pi
  return th
def compute_velocity_motion_model(prev_pose, control, dt):
  v, w = control
  x, y, theta = prev_pose
  if -1e-10 <= w <= 1e-10:      
    next_theta = theta
    next_x = x + v*np.sin(theta)*dt
    next_y = y + v*np.cos(theta)*dt
  else:
    next_theta = theta + w*dt
    next_x = x + v/w *( -np.sin(theta) + np.sin(next_theta))
    next_y = y + v/w *( np.cos(theta) - np.cos(next_theta))
  next_theta = wrapToPi(next_theta)
  return [next_x, next_y, next_theta]
def compute_inverse_velocity_motion_model(robot,target,dt):
    
    x,y,theta = robot
    next_x, next_y, next_theta = target
    
    u = 0.5 * ( (x-next_x) * np.cos(theta) + (y-next_y)*np.sin(theta) ) / ((y-next_y)*np.cos(theta) - (x-next_x)*np.sin(theta))
    star_x = (x+next_x)/2 + u*(y-next_y)
    star_y = (y+next_y)/2 + u*(next_x-x)
    star_r = ((x-star_x)**2 + (y-star_y)**2)**.5
    delta_theta = np.arctan2(next_y-star_y,next_x-star_x) - np.arctan2( y -star_y, x -star_x)
    v = delta_theta/dt * star_r
    w =  delta_theta/dt
    return [v, w] 

def compute_odometry_motion_model(robot, target):
  x, y, theta = robot
  x_t, y_t, theta_t = target

  rotation_1 = np.arctan2(y_t-y, x_t - x) - theta 
  translation = ((x-x_t)**2 + (y-y_t)**2)**.5
  rotation_2 = wrapToPi(theta_t - theta - rotation_1)

  return rotation_1, translation, rotation_2
def compute_true_measurement(robot, landmark):
  x,y, theta = robot
  landmark_x, landmark_y = landmark
  range_mea = ((landmark_x - x) ** 2 + (landmark_y - y) ** 2) ** 0.5
  bearing_mea = wrapToPi(np.arctan2((landmark_y-y),(landmark_x-x))-theta)
  return range_mea, bearing_mea
def compute_true_control(robot, target, velocity, angular_velocity):
  range_meas, bearing_meas = compute_true_measurement(robot, target)
  if abs(bearing_meas)<.1: 
    return velocity,0
  else:
    direction = 1 if 0<=bearing_meas <=np.pi else -1
    return 0,direction * angular_velocity

def compute_actual_measurement(measurement, sensor_limits, sensor_noise_std):
  range_mea, bearing_mea = measurement
  if abs(bearing_mea)>sensor_limits['bearing'] or range_mea > sensor_limits['range']:
    return None, None
  return np.random.normal(range_mea,sensor_noise_std['range']),np.random.normal(bearing_mea,sensor_noise_std['bearing'])

class Sensor():
    def __init__(self, name, update_period, limits = {'range':1,'bearing':np.pi/4}, noise_std = {'range':1, 'bearing': np.pi/36}):
      self.limits = limits
      self.noise_std = noise_std
      self.name = name
      self.update_period = update_period

class Sim():
  def __init__(self, sensors, landmarks,velocity, angular_velocity , robot_motion_std):
    self.robot_motion_std = robot_motion_std # all in meter and radian
    self.sensors = sensors
    self.dt = 1
    self.t = 0

    self.velocity, self.angular_velocity = velocity, angular_velocity 

    self.robot = [0,0,0]
    self.robot_history = []
    self.landmarks = landmarks

  def set_target(self, target: "tuple[float]"):
    self.target = target #x,y
  def increment_time(self):
    self.t += self.dt

  def move_robot_and_read_control(self): 
    v, w = compute_true_control(self.robot, self.target,self.velocity, self.angular_velocity)
    w = wrapToPi(w)
    computed_control = (v,w) #theoretical input

    v += np.random.normal(0, self.robot_motion_std['v']) 
    w += np.random.normal(0, self.robot_motion_std['w']) 
    w = wrapToPi(w)
    actual_control = (v,w)

    self.robot_history.append(copy.deepcopy(self.robot))
    self.robot = compute_velocity_motion_model(self.robot, actual_control, self.dt)
    return computed_control
  
  def read_measurement(self):
    true_measurements = {}
    actual_measurents = []
    for key, landmark_pose in list(self.landmarks.items()):
      true_measurement = compute_true_measurement(self.robot,landmark_pose)
      true_measurements[key] = true_measurement
    
    for sensor in self.sensors:
      if self.t % sensor.update_period != 0: continue
      for key, measurement in list(true_measurements.items()):
        range_mea, bearing_mea = compute_actual_measurement(measurement,sensor.limits, sensor.noise_std)
        if range_mea:
          actual_measurents.append((key, range_mea, bearing_mea, sensor.noise_std['range'], sensor.noise_std['bearing'], \
            sensor.limits['range'], sensor.limits['bearing']))
    return actual_measurents


def is_close(robot,target,acceptable_range, acceptable_bearing):
  measurement = compute_true_measurement(robot,target)
  measurement = compute_actual_measurement(measurement,{'range':acceptable_range,'bearing':acceptable_bearing},{'range':0,'bearing':0})
  if measurement[0]: return True
  return False




def runSim():
  plot_data = []
  NUM_STEPS = 500
  sensors = [Sensor('sensor0',5)]
  landmarks = [(1,0),(2,2),(4,3),\
                (5,5),(6,7),(8,9)]
  landmarks = {str(idx):landmark for idx,landmark in enumerate(landmarks)}        
  print("landmarks",landmarks)
  velocity, angular_velocity  = .2,np.pi/10
  robot_motion_std = {'v':velocity*.1, 'w': angular_velocity*.1}
  sim = Sim(sensors,landmarks,velocity, angular_velocity,robot_motion_std )
  noise_start = {'x':.5,'y':.5}
  estimated_init_landmarks = \
    {key: (\
      np.random.normal(x,noise_start['x']),\
      np.random.normal(y,noise_start['y']))\
      for key, (x,y) in list(sim.landmarks.items())}
  print("initial landmark",estimated_init_landmarks)
  n = 10

  params = {
    'initial_pose':np.array(sim.robot),
    'num_landmarks': len(sim.landmarks),
    'x_sigma': .05**2, # it is actually v_x
    'y_sigma': .05**2, # v_y
    'theta_sigma': (np.pi/360)**2,
    'v_sigma' : None,# this parameter is useless
    'land_means': estimated_init_landmarks,
    'land_covs' : {},
    'land_default_cov': np.diag([0.01, 0.01]),
    'new_land_threshold':1.3,
    'can_change_landmark':False
  }
  curr_target = str(0)
  final_taret = str(len(sim.landmarks)-1)

  slam = FastSLAM2(n,params, np.random.default_rng())

  sim.set_target(sim.landmarks['0'])
  slam_robot_pose = None
  for i in range(NUM_STEPS):
    print('---\nstep',i)
    # plotting, and also get current slam state
    particle_poses, landmark_means, landmark_covs, best_particle_idx, landmark_idxs, landmark_maps = slam.getPoseAndLandmarks()
    slam_robot_pose = particle_poses[best_particle_idx]
    slam_landmark_pose = landmark_maps[curr_target]
    print('slam robot pose',slam_robot_pose,"\nactual robot pose", sim.robot)
    print('target',sim.target)
    print('slam landmark',landmark_means)
    frame = (particle_poses, landmark_means, landmark_covs, best_particle_idx, landmark_idxs, sim.t)

    
    # get the final target
    if is_close(slam_robot_pose, landmark_maps[final_taret], .5, np.pi/4):
      NUM_STEPS = i
      break
    plot_data.append(frame)
    # move on to new target
    if is_close(slam_robot_pose, slam_landmark_pose,.5, np.pi/4):
      curr_target = str(int(curr_target)+1)
      slam_landmark_pose = landmark_maps[curr_target]

    sim.set_target(slam_landmark_pose)
    actual_measurements = sim.read_measurement()
    location_filter = lambda x: [ item[:3] for item in x]
    print("measurements",location_filter(actual_measurements))
    for subject, range_meas, bearing_meas, range_noise, bearing_noise, sensor_range_limit, sensor_bearing_limit,  in actual_measurements:
      meas_cov = np.diag([range_noise, bearing_noise])
      slam.addMeasurement((range_meas, bearing_meas), meas_cov, sensor_range_limit, sensor_bearing_limit, subject)
    
    actual_control = sim.move_robot_and_read_control()
    print("control",actual_control)
    slam.addControl(actual_control, sim.t)
    sim.increment_time()

  #plotting 
  landmarsName =  [idx for idx, (x,y) in list(sim.landmarks.items())]
  landmarksGroundtruth = np.array([np.array([x,y]) for idx, (x,y) in list(sim.landmarks.items())])
  print("end of sim","actual landmarks",sim.landmarks, "slam robot pose",slam_robot_pose,"actual robot pose", sim.robot)
  plotter.plot(n,plot_data,sim.robot_history,landmarksGroundtruth,landmarsName, NUM_STEPS,KNOWN_CORRESPONDENCES=True,PLOT_AVG=False)
  
runSim()

