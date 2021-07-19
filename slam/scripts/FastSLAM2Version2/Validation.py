import warnings
warnings.filterwarnings("ignore")
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=UserWarning) 
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import numpy as np
# Set up animation
def plot(n,plot_data,groundtruth_path_data,landmarksGroundtruth,plot_avg=False,save=False):
  fig = plt.figure()
  ax = fig.add_subplot(111)  
  ax.plot(landmarksGroundtruth[:, 0], landmarksGroundtruth[:, 1], 'cx',label='true landmark')
  
  num_steps = len(plot_data)

  particles, = ax.plot([], [], linestyle='None', marker='o', color='gold',label='est motion')
  best_particle_landmarks, = ax.plot([], [], 'mo', label='est landmark')
  
  groundtruth_path_x = []
  groundtruth_path_y = []
  groundtruth_path, = ax.plot([], [], 'bo',label='true path')
  
  best_particle_path_x = []
  best_particle_path_y = []
  best_particle_path, = ax.plot([], [], 'r-',label='est path')

  steps = ax.text(3, 6, "Step = 0 / " + str(num_steps), horizontalalignment="center", verticalalignment="top")
  ax.legend()
  
  def init():
    ax.set_title("Num steps: " + str(num_steps) + ", Num particle: "+ str(n))
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps

  def update(frame):
    t, best_particle_idx, particle_poses, landmark_idxs, landmark_means, landmark_covs,= plot_data[frame]

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
    groundtruth_path_x.append(groundtruth_path_data[t][0])
    groundtruth_path_y.append(groundtruth_path_data[t][1])
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

    # Return changed artists?
    return best_particle_path, groundtruth_path, particles, best_particle_landmarks, steps
  
  anim = animation.FuncAnimation(fig, update, frames=range(0, num_steps, max(1,int(num_steps*.001))), init_func = init, blit=False, interval = 33, repeat=False)
  fig.tight_layout(rect=[0, 0.03, 1, 0.95])
  plt.show()
  if save:
    print('Saving animation...')
    f = r'../animations/FastSLAM2_3.gif'
    writergif = animation.PillowWriter(fps=30)
    anim.save(f, writer=writergif)
    print('Animation Saved!')

def evaluate(plot_data,groundtruth_path_data,landmarksGroundtruth,landmarksNames):

  # sqrted differences between the (pose-landmark distance) 
  # for ground truth and estimation 
  sqrt_pose_landmark_dist_error = [] 

  for time, best_particle_idx, particle_poses, landmark_idxs, landmark_means, landmark_covs in plot_data:
        if len(landmark_means)==0:
          continue
        # ground truth data
        ground_truth_pose_landmark_dist = []
        estimated_pose_landmark_dist = []
        robot_x = groundtruth_path_data[time][0]
        robot_y = groundtruth_path_data[time][1]
        robot_angle = groundtruth_path_data[time][2]  
        for idx, (landmark_x, landmark_y) in zip(landmarksNames,landmarksGroundtruth):
          if idx in landmark_idxs:
            range_meas = ((robot_x - landmark_x) ** 2 + (robot_y - landmark_y) ** 2) ** 0.5
            ground_truth_pose_landmark_dist.append(range_meas)
        # fast slam data
        best_particle_pose = particle_poses[best_particle_idx]
        best_particle_x = best_particle_pose[0]
        best_particle_y = best_particle_pose[1]      
        for landmark_x, landmark_y in landmark_means:
          range_meas = ((best_particle_x - landmark_x) ** 2 + (best_particle_y - landmark_y) ** 2) ** 0.5
          estimated_pose_landmark_dist.append(range_meas)      
        ground_truth_pose_landmark_dist = np.array(ground_truth_pose_landmark_dist)
        estimated_pose_landmark_dist = np.array(estimated_pose_landmark_dist)
        sqrt_pose_landmark_dist_error.append(np.average((ground_truth_pose_landmark_dist-estimated_pose_landmark_dist)**2))
  sqrt_pose_landmark_dist_error = np.array(sqrt_pose_landmark_dist_error)
  print("sqrt_pose_landmark_dist_error",sum(sqrt_pose_landmark_dist_error**.5))
  print("avg",np.mean(sqrt_pose_landmark_dist_error**.5))
  print("median",np.median(sqrt_pose_landmark_dist_error**.5))
  print("final",sqrt_pose_landmark_dist_error[-1]**.5)
  plt.plot(sqrt_pose_landmark_dist_error**.5)
  plt.show()