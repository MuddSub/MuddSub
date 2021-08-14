import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import numpy as np
import pandas as pd
from dataclasses import dataclass

def safe_inverse(matrix, log = print, msg = 'Matrix was singular, using pseudo inverse'):
  try: 
    return np.linalg.inv(matrix)
  except np.linalg.LinAlgError:
    log(msg)
    return np.linalg.pinv(matrix)

def wrap_to_pi(th):
  '''Wraps its argument between [-pi, pi] element wise'''
  return ((th + np.pi) % (2 * np.pi)) - np.pi

def plot_df(history, groundtruth_path_data, landmarks_groundtruth, save_to=None, plot_avg = False, msg = ''):
  fig = plt.figure()
  ax = fig.add_subplot(111)
  ax.plot(landmarks_groundtruth[:, 0], landmarks_groundtruth[:, 1], 'cx', label='true landmark')

  num_particles = len(history['particle_poses'][0])
  num_steps = len(history)

  particles, = ax.plot([], [], linestyle='None', marker='o', color='gold', label='est motion')
  best_particle_landmarks, = ax.plot([], [], 'mo', label='est landmark')

  groundtruth_path_x = []
  groundtruth_path_y = []
  groundtruth_path, = ax.plot([], [], 'bo', label='true path')

  best_particle_path_x = []
  best_particle_path_y = []
  best_particle_path, = ax.plot([], [], 'r-', label='est path')

  steps = ax.text(3, 6, "Step = 0 / " + str(num_steps), horizontalalignment="center", verticalalignment="top")
  if "num_measurements" in history:
    num_measurements = ax.text(3, 7, "Num measurements = {}".format(0), horizontalalignment="center", verticalalignment="top")
  else:
    num_measurements = None
  ax.legend()

  def init():
    nonlocal msg
    print(msg)
    if msg!='':
      msg += '   '
    ax.set_title(msg+"Num steps: " + str(num_steps) + ", Num particle: " + str(num_particles))
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps, num_measurements

  def update(frame):
    t = history['timestamp'][frame]
    best_particle_idx = history['best_particle_idx'][frame]
    particle_poses = history['particle_poses'][frame]
    landmark_names = history['landmark_names'][frame]
    landmark_means = history['landmark_means'][frame]
    landmark_covs = history['landmark_covs'][frame]

    if frame == 0:
      groundtruth_path_x.clear()
      groundtruth_path_y.clear()
      best_particle_path_x.clear()
      best_particle_path_y.clear()

    # Plot best particle path
    if plot_avg:
      avg_particle_pose = np.average(particle_poses, axis=0)
      best_particle_path_x.append(avg_particle_pose[0])
      best_particle_path_y.append(avg_particle_pose[1])
    else:
      best_particle_pose = particle_poses[best_particle_idx]
      best_particle_path_x.append(best_particle_pose[0])
      best_particle_path_y.append(best_particle_pose[1])
    best_particle_path.set_data(best_particle_path_x, best_particle_path_y)

    # Plot groundtruth path
    groundtruth_path_x.append(groundtruth_path_data[frame][0])
    groundtruth_path_y.append(groundtruth_path_data[frame][1])
    groundtruth_path.set_data(groundtruth_path_x, groundtruth_path_y) # can we set them directly> groundtruth_path[:t,0]

    # Plot other particles poses
    particles.set_data(particle_poses[:, 0], particle_poses[:, 1])

    # Plot best particle landmarks
    if len(landmark_means) > 0:
      best_particle_landmarks.set_data(landmark_means[:, 0], landmark_means[:, 1])

    # Update title
    if frame/num_steps > .9:
      steps.set_text('')
    else:
      steps.set_text("Step = " + str(frame) + " / " + str(num_steps))

    # Update the number of measurements text
    if "num_measurements" in history:
      num_measurements.set_text("Num measurements = {}".format(history["num_measurements"][frame]))

    # Return changed artists?
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps, num_measurements

  anim = animation.FuncAnimation(fig, update, frames=range(0, num_steps, max(1, int(num_steps * 0.01))), init_func = init, blit=False, interval = 33, repeat=False)
  fig.tight_layout(rect=[0, 0.03, 1, 0.95])
  plt.show()
  if save_to is not None:
    print('Saving animation to {}...'.format(save_to))
    f = save_to
    writergif = animation.PillowWriter(fps=30)
    anim.save(f, writer=writergif)
    print('Animation Saved!')
