import numpy as np
from Particle import *

class FastSLAM2():
  def __init__(self, n, params, seed=0):
    self.random = np.random.default_rng(seed)
    self.data_input = None
    self.num_particles = n
    self.particles = []
    pass

  def createParticles(self, n):


  def run(self):
    # propage motion
    control, meas, meas_cov, landmark_key, dt = None, None, None, None, .1

    # correct and weights
    weights = []
    for particle in self.particles:
      weight =  particle.propagateMotion(control, meas, meas_cov, landmark_key, dt)
      weights.append(weight)
    weights = np.array(weights)

    # resampling
    new_particle_idx_ls = self.random.choice(range(self.num_particles),replace=True,p=weights)
    new_particles = []
    for idx in new_particle_idx_ls:
      # deep copy self.particle[idx] to new_particles
      pass
    pass

  def evalaute(self):
    pass
