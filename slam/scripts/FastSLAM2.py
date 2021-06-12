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
      self.params['initial_pose'] = np.array([0, 0, 0])
      self.params['num_landmarks'] = 0
      self.params['v_sigma'] = 0.04
      self.params['theta_sigma'] = 0.0125
      self.params['new_land_threshold'] = .5
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
    
    self.meas_ls = []
    self.meas_cov_ls = []
    self.sensor_range_ls = []
    self.correspondences = []
    self.known_correspondences = False
    
  def createParticles(self, n):
    for i in range(n):
      self.particles.append(Particle(i, self.params, random=self.random))
    print(self.params['initial_pose'])

  def addControl(self, control, time):
    #if len(self.meas_ls) > 0:
    #  print("num meas before update", len(self.meas_ls))
    #print("num landmark for particle 0:", len(self.particles[0].landmarks))
    self.measurementUpdate()
    # only resample if there are more than one particle 
    # have this check bc a particle can have really small weights and ~0/~0 will be Nan
    if len(self.particles) > 1 and len(self.meas_ls) > 0:
      self.resampling()
    self.motionUpdate(control, time)
    self.meas_ls, self.meas_cov_ls, self.sensor_range_ls, self.correspondences = [], [], [], []

  def addMeasurement(self, meas, meas_cov, sensor_range, correspondence = None):
    self.meas_ls.append(meas)
    self.meas_cov_ls.append(meas_cov)
    self.sensor_range_ls.append(sensor_range)
    
    self.known_correspondences = (correspondence != None)
    
    if self.known_correspondences:
      self.correspondences.append(correspondence)

  def motionUpdate(self, control, time):
    # propagate motion
    dt = max(time - self.prev_t, 0.000001)
    if (time - self.prev_t == 0):
      print("dt is 0")
    for idx, particle in enumerate(self.particles):
      particle.motionUpdate(control, dt)
    self.prev_t = time

  def measurementUpdate(self):
    # print("Known correspondences:", self.known_correspondences)
    for idx, particle in enumerate(self.particles):
      particle.measurementUpdate(self.meas_ls, self.meas_cov_ls, self.sensor_range_ls, self.known_correspondences, self.correspondences)


  def resampling(self):
    #  Collect weight
    self.weights = np.array([particle.weight for particle in self.particles])
    self.weights = self.weights / np.sum(self.weights)
    for idx, particle in enumerate(self.particles):
      particle.accumulated_weight += self.weights[idx]

    # resampling
    try:
      new_particle_idx_ls = self.random.choice(list(range(self.num_particles)), size=self.num_particles,replace=True, p=self.weights)
      
    except:
      print("Particle weights are NaN")
      print(self.weights)
      raise
      # new_particle_idx_ls = [range(self.num_particles)]

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
    