import gym
from gym import error, spaces, utils
from gym.utils import seeding
from models import *
from camera_listener import *
import numpy as np
import rospy

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
        self.current_position = self.robot_init[:3]+self.robot_init[-1]
        # x,y,theta
        # x -> move forward [-1,1]
        # y -> depth [-1,1] 
        # theta -> submarine angle [-1,1] 
        self.action_space = gym.spaces.Box(low= -1, high = 1, shape = (4,), dtype=np.float32)

    def _take_action(self,action):
        # action is a 3-tuple [-1,1],[-1,1],[-1,1],[-1,1]
        scale = 0.1
        self.current_position = [action[i]*scale+self.current_position[i] for i in range(len(action))]
        x,y,z,yaw = self.current_position
        roll = 0
        pitch = 0
        odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
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
        state = np.concatenate(pred_left,pred_right)
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
        odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
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
