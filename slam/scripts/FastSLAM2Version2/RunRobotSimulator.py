import copy
from FastSLAM2 import FastSLAM2
from RobotPhysics2D import RobotPhysics2D
import numpy as np
from Validation import plot_data, evaluate 
from Util import wrap_to_pi
from RobotSimulatorModels import RobotSimulator, Sensor
from Models import Meas,FastSLAM2Parameters, LandmarkConstants
from abc import ABC, abstractmethod

class RobotSimulatorRunner(ABC):
  def __init__(self):
    ### number of steps
    self.num_steps = 500

    ### environment setups    
    sensors = [Sensor('sensor0',5)]
    
    landmarks = [(1,0),(2,2),(4,3),\
                  (5,5),(6,7),(8,9)]
    landmarks = {str(idx):landmark for idx,landmark in enumerate(landmarks)}        
    
    velocity, angular_velocity  = .2,np.pi/10
    robot_motion_std = {'v':velocity*.1, 'w': angular_velocity*.1}
    random = np.random.default_rng()
    
    noise_start = {'x':.5,'y':.5}
    estimated_init_landmarks = {key: 
      (
        np.array([
          random.normal(x, noise_start['x']),
          random.normal(y, noise_start['y'])
        ]),
        None
      )
      for key, (x,y) in list(landmarks.items())}
    print(estimated_init_landmarks)
    initial_pose = np.array([0,0,0])
    default_pose_cov = np.diag([.05**2,.05**2,(np.pi/360)**2])
    
    self.num_particles = 10
    landmark_constants = LandmarkConstants()
    landmark_constants.new_landmark_threshold = 1.1
    self.params = FastSLAM2Parameters(
      num_particles = self.num_particles,
      is_landmarks_fixed = True, 
      initial_landmarks = estimated_init_landmarks,
      landmark_constants = landmark_constants
    )

    self.sim = RobotSimulator(sensors,landmarks, velocity, angular_velocity,robot_motion_std,random,initial_pose, default_pose_cov )

    robot_physics = RobotPhysics2D(random, initial_pose, default_pose_cov)
    self.slam = FastSLAM2(robot_physics, parameters = self.params, random = random)
    
    self.plot_data = []
    self.close_enough_distantce = .5
    self.close_enough_bearing = np.pi/4

  def initialize_landmarks(self):
    self.curr_target = str(0)
    self.final_target = str(len(self.sim.landmarks)-1)
    self.sim.set_target(self.sim.landmarks[self.curr_target])

    self.slam_robot_pose = None
    self.slam_next_landmark_pose = None
    self.landmark_maps = None

  def _log(self, *msg):
    print('+ Robot Simulator: ', *msg)

  def update_slam_snapshot(self,i):
    self._log('------------')
    self._log('step',i)

    # plotting, and also get current slam state
    frame = t , best_particle_idx, particle_poses, landmark_idxs, landmark_means, landmark_covs = self.slam.get_pose_and_landmarks_for_plot()
    self.landmark_maps = self.slam.get_landmark_map()

    self.slam_robot_pose = particle_poses[best_particle_idx]
    self.slam_next_landmark_pose = self.landmark_maps[self.curr_target]

    self._log('slam robot pose',self.slam_robot_pose,"\nactual robot pose", self.sim.robot_pose)
    self._log('slam landmarks',landmark_means)
    self._log('target',self.sim.target)

    self.plot_data.append(frame)

  def is_terminated(self):

    return self.sim.robot.is_close(self.slam_robot_pose, self.landmark_maps[self.final_target],self.close_enough_distantce, self.close_enough_bearing)

  def update_target(self):
    # move on to new target
    if self.sim.robot.is_close(self.slam_robot_pose, self.slam_next_landmark_pose,self.close_enough_distantce, self.close_enough_bearing):
      self.curr_target = str(int(self.curr_target)+1)
      self.slam_next_landmark_pose = self.landmark_maps[self.curr_target]

    self.sim.set_target(self.slam_next_landmark_pose)
    
  def update_meas(self):
    actual_measurements = self.sim.read_measurement()
    location_filter = lambda x: [ item[:3] for item in x]
    self._log("measurements",location_filter(actual_measurements))
    for subject, range_meas, bearing_meas, range_noise, bearing_noise, sensor_range_limit, sensor_bearing_limit,  in actual_measurements:
      meas_cov = np.diag([range_noise, bearing_noise])
      meas = Meas((range_meas, bearing_meas),meas_cov,(sensor_range_limit, sensor_bearing_limit), subject)         
      self.slam.add_measurement(meas)

  def update_control(self):
    actual_control = self.sim.move_robot_and_read_control()
    self._log("control",actual_control)
    self.slam.add_control(actual_control, self.sim.t)
    self.sim.increment_time()

  def plot(self):
    landmark_names =  [idx for idx, (x,y) in list(self.sim.landmarks.items())]
    landmarks_ground_truth = np.array([np.array([x,y]) for idx, (x,y) in list(self.sim.landmarks.items())])
    self._log("end of sim","actual landmarks",self.sim.landmarks, "slam robot pose",self.slam_robot_pose,"actual robot pose", self.sim.robot_pose)
    plot_data(self.slam.num_particles,self.plot_data,self.sim.robot_history,landmarks_ground_truth, plot_avg=False)

  def run(self):
    self.initialize_landmarks()
    for i in range(self.num_steps):
      self.update_slam_snapshot(i)
      if self.is_terminated(): break
      self.update_meas()
      self.update_control()
      self.update_target()
    self.plot()
  
runner = RobotSimulatorRunner()
runner.run()