import copy
from FastSLAM2 import FastSLAM2
from SimAlfie2D import SimAlfie2D
import numpy as np
from Validtion import plot_data, evaluate 
from Util import wrapToPi


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
  num_particles = 10
  params = FastSLAM2Parameters(
    num_particles = num_particles,
    is_landmarks_fixed = True, 
    new_landmark_threshold = 1.3,
    initial_pose = np.array(sim.robot_pose),
    default_pose_cov = np.array([.05**2,.05**2,(np.pi/360)**2]),
    initial_landmarks = estimated_init_landmarks
  )
  curr_target = str(0)
  final_taret = str(len(sim.landmarks)-1)

  slam = FastSLAM2(parameters = params, random = np.random.default_rng())

  sim.set_target(sim.landmarks['0'])
  slam_robot_pose = None
  for i in range(NUM_STEPS):
    print('---\nstep',i)
    # plotting, and also get current slam state
    frame = t , best_particle_idx, particle_poses, landmark_idxs, landmark_means, landmark_covs = slam.getPoseAndLandmarksForPlot()

    slam_robot_pose = particle_poses[best_particle_idx]
    slam_landmark_pose = landmark_maps[curr_target]
    print('slam robot pose',slam_robot_pose,"\nactual robot pose", sim.robot_pose)
    print('target',sim.target)
    print('slam landmark',landmark_means)
    

    
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

