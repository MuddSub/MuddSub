import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import sys
sys.path.append('/home/elip/catkin_ws/src/MuddSub/RL/object_detection/PyTorch-YOLOv3/')
print(os.listdir(sys.path[-1]))
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
    print('i got here')
    imageGen = ImageListener()
    bridge = CvBridge()
    def get_image_left(msg):
        imageGen.left_image = np.array(bridge.imgmsg_to_cv2(msg))
        print(len(imageGen.left_image))
        img = imageGen.left_image.copy()
        noise = cv2.randn(img,(0,0,0),(50,50,50))
        #imageGen.left_image+=noise
    def get_image_right(msg):
        imageGen.right_image = np.array(bridge.imgmsg_to_cv2(msg))
        img = imageGen.right_image.copy()
        noise = cv2.randn(img,(0,0,0),(0,0,0))
        #imageGen.right_image+=noise
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

        setImageSubscriber(self.imageGen)
        self.publisher = setStatePublisher()
        self.robot_init = [0,3,0,0,0,0]
        self.current_step = 0

        # x,y,z, yaw
        self.current_position = self.robot_init[:3]+[self.robot_init[-1]]

        self.action_space = gym.spaces.Discrete(6)

    def _take_action(self, action):

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
        print(odom)
        self.publisher.publish(odom)

    def _next_observation(self):
        # CONSIDER ADDING CURRENT POSITION TO STATE, 6 dimensional
        img_left, img_right = self.getImages()
        print(len(img_left))
        print("prediction",self.model(img_left))
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
        x,y,z,roll,pitch,yaw = self.robot_init
        odom_quat = self.euler_to_quaternion(roll, pitch, yaw)
        odom = Odometry()
        odom.pose.pose = Pose(Point(x, y, z), odom_quat)
        self.publisher.publish(odom)
        self.loopRate.sleep()

        # Return initial state
        state =self._next_observation()
        return stateself

    def render(self, mode='human', close=False):
        pass

    def getImages(self):
        img_left = self.imageGen.left_image
        img_right = self.imageGen.right_image
        return img_left, img_right

    def euler_to_quaternion(self, roll, pitch, yaw):

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        # qw = 1
        return rot_quat
