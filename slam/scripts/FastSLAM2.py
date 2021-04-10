import numpy as np
import copy 
from Particle import *

class FastSLAM2():
  '''
  params: a dictionary. contains keys:
    xRange: 2 tuple containing the minimum and maximum x values for the robot
    yRange: 2 tuple containing the minimum and maximum y values for the robot
    and all of the parameters used by the Particle class
  '''
  def __init__(self, n, params, seed=0):
    self.random = np.random.default_rng(seed)
    self.data_input = None
    self.num_particles = n
    self.particles = []
    self.history = []
    self.xRange = params['xRange']
    self.yRange = params['yRange']
    self.particleParams = {}
    self.particleParams['initial_pose'] = np.array([0, 0, 0, 0, 0, 0, 0])
    self.meas_cov = np.diag([0.075, 0.025])
    self.prev_t = 0
    
  def createParticles(self, n):
    for i in range(n):
      self.particles.append(Particle(i, self.particleParams))

  def measurementUpdate(self, frame):
  # def measurementUpdate(self, control, meas, meas_cov, t):
    frameTime, groundtruth, measurements, odometry = frame
    measurements_cov = [self.meas_cov] * len(measurements)

    dt = max(frameTime - self.prev_t, 0.000001)
    if (frameTime-self.prev_t == 0):
      print("dt is 0")
    
    # propagate motion
    for idx, particle in enumerate(self.particles):
      for control in odometry:
        particle.propagateMotion(control, dt)
    
    #  Collect weight
    self.weights = np.ones(len(self.particles))
    for idx, particle in enumerate(self.particles):
      for meas, meas_cov in zip(measurements, measurements_cov):
        self.weights[idx] *= particle.updateEKFs(meas, meas_cov)
    self.weights /= np.sum(self.weights)

    # resampling
    new_particle_idx_ls = self.random.choice(list(range(self.num_particles)), replace=True, p=self.weights)
    new_particles = []
    for idx in new_particle_idx_ls:
      new_particles.append(copy.deepcopy(self.particles[idx]))
    self.particles  = new_particles
    self.prev_t = frameTime
    return self.store(self.particles, frameTime)
    
  def store(self, particles, t):
    poses =  np.zeros(len(particles))
    for idx, particle in enumerate(particles):
      poses[idx] = particle.pose
      for landmark in particle.landmark:
        pass
    avg_pose = np.average(poses)
    hist = dict()
    hist['time'] = t
    hist['pose'] = avg_pose
    return hist