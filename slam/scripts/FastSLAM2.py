import numpy as np
import copy 
from Particle import *

class FastSLAM2():
  '''
  params: a dictionary. contains keys:
    # xRange: 2 tuple containing the minimum and maximum x values for the robot
    # yRange: 2 tuple containing the minimum and maximum y values for the robot
    and all of the parameters used by the Particle class
  '''
  def __init__(self, n,  params=None, random=None):
    if params is not None:
      self.params = params
    else:
      self.params = {}
      self.params['initial_pose'] = np.array([0, 0, 0, 0, 0, 0, 0])
      self.params['num_landmarks'] = 0
      self.params['v_sigma'] = 0.04
      self.params['omega_sigma'] = 0.05
      self.params['theta_sigma'] = 0.0125
      self.params['prob_threshold'] = .5
      self.params['sensor_range'] = 10
      #TODO Figure out how to get variance for x and y
      self.params['x_sigma'] = 1
      self.params['y_sigma'] = 1
      
    # self.random = np.random.default_rng(seed)
    self.random = random
    self.data_input = None
    self.num_particles = n
    self.particles = []
    self.history = []
    # self.xRange = params['xRange']
    # self.yRange = params['yRange']
    
    '''
    params: a dictionary. contains keys:
      initial_pose: initial pose vector
      num_landmarks: initial number of landmarks
      v_sigma: velocity variance
      omega_sigma: angular velocity variance
      theta_sigma: angle variance
    '''
    
    self.prev_t = 0
    self.createParticles(self.num_particles)
    
  def createParticles(self, n):
    for i in range(n):
      self.particles.append(Particle(i, self.params, random=self.random))
    print(self.params['initial_pose'])

  def propagateMotion(self, control):
    # propagate motion
    #print("FastSLAM control", control)
    time, vx, vy, theta_imu, omega_imu = control
    dt = max(time - self.prev_t, 0.000001)
    if (time-self.prev_t == 0):
      print("dt is 0")
    for idx, particle in enumerate(self.particles):
      particle.propagateMotion((vx, vy, theta_imu, omega_imu), dt)
    self.prev_t = time
    return self.get_all_poses()

  def updateMeasurement(self, meas, meas_cov):
    #print("meas", meas)
    time, range_meas, bearing_meas = meas
    
    #  Collect weight
    self.weights = np.ones(len(self.particles))
    for idx, particle in enumerate(self.particles):
      self.weights[idx] *= particle.updateEKFs((range_meas, bearing_meas), meas_cov)
    self.weights /= np.sum(self.weights)

    # resampling
    new_particle_idx_ls = self.random.choice(list(range(self.num_particles)), size=self.num_particles,replace=True, p=self.weights)
    new_particles = []
    for idx in new_particle_idx_ls:
      new_particles.append(copy.deepcopy(self.particles[idx]))
      new_particles[-1].random = self.random
    self.particles  = new_particles
    return self.get_all_poses() #self.compute_avg(self.particles, time)
    
  def compute_avg(self, particles, t):
    poses =  []
    
    for idx, particle in enumerate(particles):
      poses.append(particle.pose)
      for landmark in particle.landmarks:
        pass
    avg_pose = np.average(np.array(poses),axis=0)
    #print(hist)
    return avg_pose
  def get_all_poses(self):
    poses =  []   
    for idx, particle in enumerate(self.particles):
      poses.append(particle.pose[:2])
    return poses
    