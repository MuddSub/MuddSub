import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import sys
sys.path.append('/home/elip/catkin_ws/src/MuddSub/RL/object_detection/PyTorch-YOLOv3/')
print(os.listdir(sys.path[-1]))
import rospy
from models import *
from camera_listener import *
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class MuddSubEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(MuddSubEnv, self).__init__()
        loopRate = rospy.Rate(10)
        model_checkpoint = '../checkpoints/1.pth'
        self.gate_position = (3,3,6)
        self.model = Darknet(model_checkpoint)
        self.imageGen = ImageListener()
        rospy.init_node("RL_node",anonymous=True)
        setImageSubscriber(self.imageGen)
        self.publisher = setStatePublisher()
        self.robot_init = [0,3,0,0,0,0]
        self.current_step = 0

        # x,y,z, yaw
        self.current_position = self.robot_init[:3]+self.robot_init[-1]

        self.action_space = gym.spaces.Discrete(6)

    def _take_action(self,action):

        scale = 0.01    # movement amount
        degree = 20     # turn amount in degrees
        x,y,z,yaw = self.current_position

        if action == "forward":
            x += scale * np.cos(yaw)
            y += scale * np.sin(yaw)
        elif action == "backward":
            x -= scale * np.cos(yaw)
            y -= scale * np.sin(yaw)
        elif action == "turn_left":
            yaw += degree/180 * np.pi
        elif action == "turn_right":
            yaw -= degree/180 * np.pi
        elif action == "up":
            z -= scale
        elif action == "down":
            z += scale

        self.current_position = x,y,z,yaw

        roll = 0
        pitch = 0
        odom_quat = euler_to_quaternion(yaw, pitch, roll)
        odom = Odometry()
        odom.pose.pose = Pose(Point(x, y, z), odom_quat)
        self.publisher.publish(odom)

    def _next_observation(self):
        # CONSIDER ADDING CURRENT POSITION TO STATE, 6 dimensional
        img_left, img_right = getImages()
        pred_left = self.model(img_left).numpy()[0][0]
        pred_right = self.model(img_right).numpy()[0][0]
        #if pred_left/pred_right is null or something:
        #    return -1
        if pred_left == None:
          pred_left = np.array([-1,-1])
        if pred_right == None:
          pred_right = np.array([-1,-1])
        visionState = np.concatenate(pred_left,pred_right)
        state = np.concatenate(visionState, self.current_position)
        return state

    def _distanceToGate(self):
        x,y,z,_ = self.current_position
        gx,gy,gz = self.gate_position
        return (x-gx)**2+(y-gy)**2+(z-gz)**2

    def _checkAtGate(self):
        # Needs to be tuned
        self.distance_threshold = .05
        d = self._distanceToGate()
        if d < self.distance_threshold:
            # One possibility to make better is check if normal vector, vector to gate is not to far apart in angle
            return True
        return False


    def step(self, action):
        # state, reward, episode_over, info
        self._take_action(action)
        self.current_step+=1

        reward = -self._distanceToGate()
        done = self._checkAtGate()

        state = self._next_observation()
        if state == -1:
            done = True
            return None, 0, done, {}
        return state, reward, done, {}

    def reset(self):
        # Publisher sets robot state to some init, time to 0
        x,y,z,roll,pitch,yaw = robot_init
        odom_quat = euler_to_quaternion(yaw, pitch, roll)
        odom = Odometry()
        odom.pose.pose = Pose(Point(x, y, z), odom_quat)
        self.publisher.publish(odom)
        self.loopRate.sleep()

        # Return initial state
        state =self._next_observation()
        return state

    def render(self, mode='human', close=False):
        pass

    def getImages():
        img_left = imageGen.left_image
        img_right = imageGen.right_image
        return img_left, img_right
    def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
