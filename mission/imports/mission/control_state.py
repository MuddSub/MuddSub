#!/usr/bin/env python3
import rospy
import smach
from slam.msg import Map, Obstacle
from std_msgs.msg import Bool
from vision.msg import DetectionArray
from nav_msgs.msg import Odometry
from controls.msg import State
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from functools import reduce
import numpy as np

def state_callback(msg):
  pos = msg.pose.pose.position
  orientation = msg.pose.pose.orientation
  orientation = [orientation.w, orientation.x, orientation.y, orientation.z]
  r,p,y = euler_from_quaternion(orientation)
  vel = msg.twist.twist.linear
  omega = msg.twist.twist.anguler
  ControlState._current_state = np.array([pos.x, pos.y, pos.z, r, p, y, vel.x, vel.y, vel.z, omega.x, omega.y, omega.z])

class ControlState(smach.State):
  _current_state = None
  _state_subscriber = rospy.Subscriber('/slam/robot/state', Odometry, state_callback)
  _header_seq = 0

  def __init__(self):
    pass

  def spin(self, angle_offset):
    '''Requests a spin of angleOffset radians from controls'''
    setpoint = ControlState._current_state + np.array([0, 0, 0, 0, 0, angle_offset, 0, 0, 0, 0, 0, 0])
    self.publish_setpoint(setpoint)
  
  def move_forward(self, distance):
    '''Requests a movement of distance meters forward relative to the robot's bearing'''
    setpoint = ControlState._current_state + np.array([distance * np.cos(ControlState._current_state[5]), distance * np.sin(ControlState._current_state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    self.publish_setpoint(setpoint)

  def publish_setpoint(self, setpoint):
    state = State()
    state.header = Header(ControlState._header_seq, rospy.Time.now(), 'BASE')
    state.state = setpoint.tolist()
    ControlState._header_seq += 1
    ControlState._setpoint_publisher.publish(state)