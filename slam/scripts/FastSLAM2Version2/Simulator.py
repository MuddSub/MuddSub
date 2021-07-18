import copy
from FastSLAM2 import FastSLAM2
from SimAlfie2D import SimAlfie2D
import numpy as np
import plotter
from Util import wrapToPi

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

    self.robot = SimAlfie2D()
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
    'initial_pose':np.array(sim.robot_pose),
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
    print('slam robot pose',slam_robot_pose,"\nactual robot pose", sim.robot_pose)
    print('target',sim.target)
    print('slam landmark',landmark_means)
    frame = (particle_poses, landmark_means, landmark_covs, best_particle_idx, landmark_idxs, sim.t)

    
    # get the final target
    if sim.robot.is_close(slam_robot_pose, landmark_maps[final_taret], .5, np.pi/4):
      NUM_STEPS = i
      break
    plot_data.append(frame)
    # move on to new target
    if sim.robot.is_close(slam_robot_pose, slam_landmark_pose,.5, np.pi/4):
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
  print("end of sim","actual landmarks",sim.landmarks, "slam robot pose",slam_robot_pose,"actual robot pose", sim.robot_pose)
  plotter.plot(n,plot_data,sim.robot_history,landmarksGroundtruth,landmarsName, NUM_STEPS,KNOWN_CORRESPONDENCES=True,PLOT_AVG=False)
  
runSim()

