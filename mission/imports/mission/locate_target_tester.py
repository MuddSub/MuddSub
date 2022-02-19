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
# check if we found in camera

'''
Case 1: we don't see the Gate in the camera, we need to look for it
     - Gate
       - Spin around first till we see it, then we can pass to "go to target"
     - For all other task
       - Look for the path finder and move accordingly, using the bottom facing camera
       - look going until we see the next Task in the camera/map
       - shift to "go to target"

Case 2: if we see the Gate, we move to "go to target"


background info we need to be passed in:
 1. the task_name

check if the task_name == Gate
 - spin around until we see the gate
  - What should we do if we still don't see the gate after 2 rotations?
     - potential solutions, move directly forward until we see the gate and pass to "go to target".

if the task_name != Gate
 - look for the path finder
  - move directly on top of it and then keep moving 
    so that the center of our center is lined up with the direction of the path finder
  - keep moving in the last direction until we see the task and pass to "go to target"
'''

# things to do: 1. camera_name 2. import vision.msg 3. is gate capitalized


class LocateTarget(smach.State):
  def __init__(self, task_name, camera_name, min_confidence, thresholds):
    rospy.loginfo("LocateTarget init")
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], 
                              input_keys = ['isWaiting_in',],
                              output_keys = ['isWaiting_out'])
    self.locateTarget_subscriber = rospy.Subscriber('/mission/target', Bool, self.callback)
    self.detection_subscriber = rospy.Subscriber('vision/' + camera_name + '/detection_array', DetectionArray, self.detection_callback)
    self.error_subscriber =  rospy.Subscriber('/controls/robot/error',Odometry,self.error_callback)
    self.state_subscriber = rospy.Subscriber('/slam/robot/state', Odometry, self.state_callback)
    self.setpoint_publisher = rospy.Publisher('/robot_setpoint', State)
    self.camera_name = camera_name
    self.found_target = False
    self.reached_requested_position = False
    self.spin_count = 0
    self.min_confidence = min_confidence
    self.threshold = thresholds
    self.task_name = task_name # we are not doing anything which this yet!
    self.state = None
    self.numSpins = 8
    self.centered = False
    self.header_seq = 0
    rospy.loginfo("task_name is " + task_name)

  def detection_callback(self, data):
    for i in data.detections:
      if i.name == 'gate' and i.confidence > self.min_confidence:
        self.found_target = True
        self.centered = abs(i.boundingBox.center.x - 0.5) < 0.01
        break

  def error_callback(self,data):
      # thresholds[point_threshold, angle_, velocity, angular velocity]
      # point (m), quarternion( rad), xyz (m/s), angle velocity (rad/s)
      # 1. x, y, z same threshold
      # 2. roll, pitch, yaw same threshold
      point = data.pose.pose.position.x
      point = (point.x, point.y, point.z)
      orientation = euler_from_quaternion(data.pose.pose.orientation)
      velocity = data.twist.twist.linear
      velocity = (velocity.x, velocity.y, velocity.z)      
      angular_velocity = data.twist.twist.angular
      angular_velocity = (angular_velocity.x, angular_velocity.y, angular_velocity.z)
      self.reached_requested_position = self.check_threshold(point, self.threshold[0]) and \
                                        self.check_threshold(orientation, self.threshold[1]) and \
                                        self.check_threshold(velocity, self.threshold[2]) and \
                                        self.check_threshold(angular_velocity, self.threshold[3])

  def check_threshold(self, position, threshold):
    return reduce(lambda x, y: x and y, list(map(lambda x: x < threshold, position)))

  def execute(self, userdata):
    detection_subscriber = rospy.Subscriber('vision/' + self.camera_name + '/detection_array', DetectionArray, self.detection_callback)
    error_subscriber =  rospy.Subscriber('/controls/robot/error', Odometry, self.error_callback)
    if self.task_name == 'Gate':
      if self.found_target and self.centered:
        rospy.loginfo('Request to stop')
        return 'succeeded'

      # waiting for controls to move us
      elif userdata.isWaiting_in:
        if self.reached_requested_position:
          userdata.isWaiting_out = False
        return 'active'

      else:
        self.spin_count += 1
        if self.spin_count < self.numSpins:
          self.requestSpin(np.pi / 2)
        elif self.spin_count == self.numSpins:
          self.requestForwardMovement(10)
        elif self.spin_count >= self.numSpins:
          return 'abort'
        userdata.isWaiting_out = True
        return 'active'
  
  def stateCallback(self, msg):
    pos = msg.pose.pose.position
    r,p,y = euler_from_quaternion(msg.pose.pose.orientation)
    vel = msg.twist.twist.linear
    omega = msg.twist.twist.anguler
    self.state = np.array([pos.x, pos.y, pos.z, r, p, y, vel.x, vel.y, vel.z, omega.x, omega.y, omega.z])

  def requestSpin(self, angleOffset):
    '''Requests a spin of angleOffset radians from controls'''
    setpoint = self.state + np.array([0, 0, 0, 0, 0, angleOffset, 0, 0, 0, 0, 0, 0])
    self.publishSetpoint(setpoint)
  
  def requestForwardMovement(self, distance):
    '''Requests a movement of distance meters forward relative to the robot's bearing'''
    setpoint = self.state + np.array([np.cos(self.state[5]), np.sin(self.state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    self.publishSetpoint(setpoint)

  def publishSetpoint(self, setpoint):
    state = State()
    state.header = Header(self.header_seq, rospy.Time.now(), 'BASE')
    state.state = setpoint.tolist()
    self.header_seq += 1
    self.setpoint_publisher.publish(state)