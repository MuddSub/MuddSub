import numpy as np
import copy
import pandas as pd
from slam.Particle import *
from collections import namedtuple
from slam.Models import Meas, FastSLAM2Parameters, LandmarkConstants

class FastSLAM2():
  def __init__(self, robot_physics, parameters: FastSLAM2Parameters, random=np.random.default_rng()):

    self.num_particles = parameters.num_particles
    self.particles = []

    self._meas_ls: list[Meas] = []
    self._prev_t = 0
    self._random = random
    self._particles_params = {
      'random': self._random,
      'are_landmarks_fixed': parameters.are_landmarks_fixed,
      'initial_landmarks': parameters.initial_landmarks,
      'landmark_constants': parameters.landmark_constants,
      'verbose': parameters.verbose
    }
    self._localization_only = parameters.localization_only
    self._verbose = parameters.verbose
    self._robot_physics = robot_physics

    self._create_particles(self.num_particles)

  def _create_particles(self, n):
    for i in range(n):
      self.particles.append(Particle(robot_physics = self._robot_physics, particle_id=i, **self._particles_params))
    self._log('info',"fast slam 2 initial pose", self._robot_physics.initial_pose)

  def add_control(self, control, time):
    self._update_meas()
    if len(self.particles) > 1 and len(self._meas_ls) > 0: # only resample if there are more than one particle and observations 
      self._resample()
    self._update_motion(control, time)
    self._meas_ls = []

  def add_measurement(self, meas: Meas):
    self._meas_ls.append(meas)

  def _log(self, *msg):
    if self._verbose >= 1:
      print('FastSLAM2:', *msg)
  
  def _update_motion(self, control, time): #propagate motion
    dt = max(time - self._prev_t, 0.000001)
    if (time - self._prev_t == 0): self._log("warning:","dt is 0")
    for particle in self.particles:
      particle.update_motion(control, dt)
    self._prev_t = time

  def _update_meas(self):
    for particle in self.particles:
      if self._localization_only:
        particle.update_meas_localization_only(self._meas_ls)
      else:
        particle.update_meas(self._meas_ls)

  def _resample(self):    #  Collect weight
    self.weights = np.array([particle.weight for particle in self.particles])
    self.weights = self.weights / np.sum(self.weights)
    for idx, particle in enumerate(self.particles):
      particle.accumulated_weight += self.weights[idx]

    try: # resampling
      new_particle_idx_ls = self._random.choice(list(range(self.num_particles)), size=self.num_particles, replace=True, p=self.weights)
    except:
      self._log('warning', 'Particle weights are NaN',self.weights)
      new_particle_idx_ls = list(range(self.num_particles))
    self._log('particle pose',self.particles[0].pose)
    new_particles = []
    for idx in new_particle_idx_ls:
      new_particles.append(copy.deepcopy(self.particles[idx]))
      new_particles[-1]._random = self._random
    self.particles  = new_particles
    self._log('particle pose after',self.particles[0].pose)

  def get_pose_and_landmarks_for_plot(self):
    best_particle = max(self.particles, key=lambda p: p.accumulated_weight)
    particle_poses = []
    landmark_means = []
    landmark_covs = []
    landmark_names = []

    for particle_idx, particle in enumerate(self.particles):
      if particle is best_particle:
        best_particle_idx = particle_idx
      particle_poses.append(np.copy(particle.pose))

    for name, landmark in best_particle.landmarks.items():
      landmark_means.append(np.copy(landmark.mean))
      landmark_covs.append(np.copy(landmark.cov))
      landmark_names.append(name)

    snapshot = pd.DataFrame({
      'timestamp': [self._prev_t],
      'best_particle_idx': [best_particle_idx],
      'particle_poses': [np.array(particle_poses)],
      'landmark_names': [np.array(landmark_names)],
      'landmark_means': [np.array(landmark_means)],
      'landmark_covs': [np.array(landmark_covs)]
    })
    return snapshot
    
  def get_landmark_map(self, option='best'):
    landmark_map = {}
    if option == 'best':
      best_particle = max(self.particles, key=lambda p: p.accumulated_weight)
      for idx, landmark in best_particle.landmarks.items():
        landmark_map[idx] = landmark.mean
    return landmark_map