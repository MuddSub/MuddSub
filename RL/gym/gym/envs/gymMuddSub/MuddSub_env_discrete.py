#!/usr/bin/env python3
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import sys
sys.path.append('/home/elip/catkin_ws/src/MuddSub/RL/object_detection/PyTorch-YOLOv3/')
#print(os.listdir(sys.path[-1]))
import rospy
from models import *
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import cv2
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge

class ImageListener:
    left_image = None
    right_image = None

def setImageSubscriber(imageGen):

    imageGen = ImageListener()


    #rospy.init_node('image_listener', anonymous=True)
    sub_left = rospy.Subscriber('/cameras/front_left/raw', Image, get_image_left)
    sub_right = rospy.Subscriber('/cameras/front_right/raw', Image, get_image_right)

def setStatePublisher():
    publisher = rospy.Publisher('/robot_state',Odometry,queue_size=10)
    return publisher

class MuddSubEnvDiscrete(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(MuddSubEnvDiscrete, self).__init__()
        rospy.init_node("RL_node",anonymous=True)
        self.loopRate = rospy.Rate(10)
        model_checkpoint = '/home/elip/catkin_ws/src/MuddSub/RL/object_detection/PyTorch-YOLOv3/config/yolov3.cfg'
        self.gate_position = (3,3,6)
        self.model = Darknet(model_checkpoint)
        self.imageGen = ImageListener()
        bridge = CvBridge()

        def get_image_left(msg):
            self.imageGen.left_image = np.array(cv2.resize(bridge.imgmsg_to_cv2(msg), (416,416)) )

        def get_image_right(msg):
            self.imageGen.right_image = np.array(cv2.resize(bridge.imgmsg_to_cv2(msg), (416,416)) )

        rospy.Subscriber('/cameras/front_left/raw', Image, get_image_left)
        rospy.Subscriber('/cameras/front_right/raw', Image, get_image_right)

        self.publisher = setStatePublisher()
        self.robot_init = [0,0,1.5,0,0,0]
        self.current_step = 0

        # x,y,z, yaw
        self.current_position = self.robot_init[:3]+[self.robot_init[-1]]

        self.action_space = gym.spaces.Discrete(6)

    def euler_to_quaternion(self, roll, pitch, yaw):

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        # qw = 1=
        return Quaternion(list(rot_quat)[0], list(rot_quat)[1], list(rot_quat)[2], list(rot_quat)[3])


    def _take_action(self, action):

        scale = 0.5    # movement amount
        degree = 20     # turn amount in degrees
        x,y,z,yaw = self.current_position
        action2word = ['forward','backward','turn_left','turn_right','up','down']

        if action == 0: #"forward":
            x += scale * np.cos(yaw)
            y += scale * np.sin(yaw)
        elif action == 1: #"backward":
            x -= scale * np.cos(yaw)
            y -= scale * np.sin(yaw)
        elif action == 2: #"turn_left":
            yaw -= degree/180 * np.pi
        elif action == 3: #"turn_right":
            yaw += degree/180 * np.pi
        elif action == 4: #"up":
            z -= scale
        elif action == 5: #"down":
            z += scale

        self.current_position = x,y,z,yaw

        roll = 0
        pitch = 0
        odom_quat = self.euler_to_quaternion(roll, pitch, yaw)
        odom = Odometry()
        odom.pose.pose = Pose(Point(x, y, z), odom_quat)
        print("X: ", x, "Y: ", y, "Z: ", z, "Action: ", action2word[action])

        self.publisher.publish(odom)

    def _next_observation(self):
        # CONSIDER ADDING CURRENT POSITION TO STATE, 6 dimensional
        img_left, img_right = self.getImages()

        img = np.array([img_left, img_right])
        img = torch.from_numpy(img)
        img_left = torch.from_numpy(np.array([img_left]))
        img_right = torch.from_numpy(np.array([img_right]))
        img = img.permute(0, 3, 2, 1)
        img = img.float()
        img_left = img_left.permute(0, 3, 2, 1)
        img_left = img_left.float()
        img_right = img_right.permute(0, 3, 2, 1)
        img_right = img_right.float()
        #print("prediction",self.model(img)[0][0])
        pred_left = self.model(img_left)
        pred_right = self.model(img_right)
        pred_left = non_max_suppression(pred_left, conf_thres=0.5, nms_thres=0.4)[0].numpy()[0][:4]
        pred_right = non_max_suppression(pred_right, conf_thres=0.5, nms_thres=0.4)[0].numpy()[0][:4]

        print("pred_left", pred_left)
        #if pred_left/pred_right is null or something:
        #    return -1

        if pred_left.all() == None:
          pred_left = np.array([-1,-1])
        if pred_right.all() == None:
          pred_right = np.array([-1,-1])

        visionState = np.concatenate((pred_left,pred_right))
        state = np.concatenate((visionState, self.current_position))
        print("state: ", state)
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

        if state[8] <= -1 or state[8] >= 7: # run into x walls
            done = True
            print("reward: ", -1000)
            return None, -1000, done, {}

        print("reward: ", reward)
        return state, reward, done, {}

    def reset(self):
        # Publisher sets robot state to some init, time to 0
        x,y,z,roll,pitch,yaw = self.robot_init
        odom = Odometry()
        odom.pose.pose = Pose(Point(x, y, z), self.euler_to_quaternion(roll, pitch, yaw))
        self.publisher.publish(odom)
        self.loopRate.sleep()

        # Return initial state
        stateself =self._next_observation()
        return stateself

    def render(self, mode='human', close=False):
        pass

    def getImages(self):
        img_left = self.imageGen.left_image
        img_right = self.imageGen.right_image
        return img_left, img_right
