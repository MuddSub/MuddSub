import numpy as np
import copy
from Particle import *
from collections import namedtuple
from Models import MEAS
# meas_ls, meas_cov_ls, sensor_range_ls, sensor_bearing_ls, known_correspondences = False, correspondences = []

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
      self.params['can_change_landmark'] = False
      
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
    
    self.meas_ls: list[MEAS] = [] 

  def createParticles(self, n):
    for i in range(n):
      self.particles.append(Particle(i, self.params, random=self.random))
    print("fast slam 2 initial pose",self.params['initial_pose'])

  def addControl(self, control, time):
    self._updateMeas()
    if len(self.particles) > 1 and len(self.meas_ls) > 0: # only resample if there are more than one particle and observations 
      self._resample()
    self._updateMotion(control, time)
    self.meas_ls = []

  def addMeasurement(self, meas: MEAS):
    self.meas_ls.append(meas)

  def _log(self, *msg):
    print('+ FastSLAM2:', *msg)
  
  def _updateMotion(self, control, time): #propagate motion
    dt = max(time - self.prev_t, 0.000001)
    if (time - self.prev_t == 0): self._log("fast slam 2 warning: dt is 0")
    for particle in self.particles:
      particle.updateMotion(control, dt)
    self.prev_t = time

  def _updateMeas(self):
    for idx, particle in enumerate(self.particles):
      particle.updateMeas(self.meas_ls)

  def _resample(self):    #  Collect weight
    self.weights = np.array([particle.weight for particle in self.particles])
    self.weights = self.weights / np.sum(self.weights)
    for idx, particle in enumerate(self.particles):
      particle.accumulated_weight += self.weights[idx]
    try: # resampling
      new_particle_idx_ls = self.random.choice(list(range(self.num_particles)), size=self.num_particles, replace=True, p=self.weights)
    except:
      self._log('Waring', 'Particle weights are NaN',self.weights)
      new_particle_idx_ls = list(range(self.num_particles))

    new_particles = []
    for idx in new_particle_idx_ls:
      new_particles.append(copy.deepcopy(self.particles[idx]))
      new_particles[-1].random = self.random
    self.particles  = new_particles

  # TODO: SHOULD CREATE A NEW DATA STRUCT
  def getPoseAndLandmarks(self,option='best'): 
    if option=='best':
      best_particle = max(self.particles, key=lambda p: p.accumulated_weight)
      particle_poses = []
      landmark_means = []
      landmark_covs = []
      landmark_idxs = []
      landmark_map = {}
      for particle_idx, particle in enumerate(self.particles):
        if particle is best_particle:
          best_particle_idx = particle_idx
        particle_poses.append(np.copy(particle.pose))

      for idx, landmark in best_particle.landmarks.items():
        landmark_map[idx] = landmark.land_mean
        landmark_means.append(np.copy(landmark.land_mean))
        landmark_covs.append(np.copy(landmark.land_cov))
        landmark_idxs.append(idx)
      return np.array(particle_poses), np.array(landmark_means), np.array(landmark_covs), best_particle_idx, landmark_idxs,landmark_map