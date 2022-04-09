#!/usr/bin/env python3
from multiprocessing.dummy import active_children
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros
from vision.msg import DetectionArray
from controls.msg import State
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from functools import reduce
import numpy as np

class GateAction(smach.State):
  '''
  Move to Gate

  Once we detect the gate, we need to move(straight) to the gate (assuming there is no obstacles)
  We need to check our position relative to the gate as we move

  If we see the gate directly facing us, we move forward
    - Communicate with control?

  If we sdon't see the gate, we rotate and search for the gate again
    - Goes back to the previous state, locate target

  If we reached the gate, return succeeded
    - how do we determine if we have reached the gate
    - How do we differentiate reaching gate and losing gate in sight

  '''
  
  def __init__(self,camera_name, min_confidence, thresholds):
    smach.State.__init__(self, outcomes=['active', 'succeeded', 'aborted','lost_target'],
                              input_keys = ['isWaiting',],
                              output_keys = ['isWaiting'])
                            

    self.isVisible = rospy.Subscriber('vision/' + camera_name + '/detection_array', DetectionArray, self.gate_callback)
    self.reachedTarget_subscriber = rospy.Subscriber('/controls/robot/error', Odometry, self.reachedTarget_callback)
    self.state_subscriber = rospy.Subscriber('/slam/robot/state', Odometry, self.state_callback)
    self.setpoint_publisher = rospy.Publisher('/robot_setpoint', State)
    self.GATE_LENGTH = 3.048  
    
    self.camera_name = camera_name
    self.min_confidence = min_confidence
    self.threshold = thresholds

    self.gate_visible = True
    self.beam_visible = True
    self.reached_requested_pos = False

    self.range_to_beam = float('-inf')
    self.startTime = rospy.get_time()
    self.lastSearch = self.startTime
    self.header_seq = 0


  def state_callback(self, msg):
    pos = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    orientation = [orientation.w, orientation.x, orientation.y, orientation.z]
    r,p,y = euler_from_quaternion(orientation)
    vel = msg.twist.twist.linear
    omega = msg.twist.twist.angular
    self.current_state = np.array([pos.x, pos.y, pos.z, r, p, y, vel.x, vel.y, vel.z, omega.x, omega.y, omega.z])

  def gate_callback(self,data):
    gate_flag = False
    beam_flag = False
    for i in data.detections:
      if i.name == 'Gate' and i.confidence > self.min_confidence:
        gate_flag= True
      elif i.name == 'gate_center_beam' and i.confidence > self.min_confidence:
        beam_flag = True
        self.range_to_beam = i.range

    self.beam_visible = beam_flag
    self.gate_visible = gate_flag
  
  def reachedTarget_callback(self,data):
    self.lastSearch = rospy.get_time()

    point = data.pose.pose.position.x
    point = (point.x, point.y, point.z)
    orientation = data.pose.pose.orientation
    orientation = [orientation.w, orientation.x, orientation.y, orientation.z]
    orientation = euler_from_quaternion(orientation)
    velocity = data.twist.twist.linear
    velocity = (velocity.x, velocity.y, velocity.z)      
    angular_velocity = data.twist.twist.angular
    angular_velocity = (angular_velocity.x, angular_velocity.y, angular_velocity.z)
    self.reached_requested_position = self.check_threshold(point, self.threshold[0]) and \
                                      self.check_threshold(orientation, self.threshold[1]) and \
                                      self.check_threshold(velocity, self.threshold[2]) and \
                                      self.check_threshold(angular_velocity, self.threshold[3])

  def check_threshold(self, position, threshold):
    return reduce(lambda x, y: x and y, map(lambda x: x < threshold, position))
  
  def execute(self, ud):
    if self.startTime - self.lastSearch > 60:
      return 'aborted'

    elif abs(self.range_to_beam - 1) < 0.1:
      self.requestForwardMovement(0)

      self.requestAnyMovement("Right", self.GATE_LENGTH / 4)
      self.requestForwardMovement(1)
      return 'succeeded'

    elif ud.isWaiting and (self.bem_visible or self.gate_visible):
      if self.reached_requested_pos:
          ud.isWaiting = False
      return 'active'

    elif not ud.isWaiting and (self.beam_visible or self.gate_visible):
      self.requestForwardMovement(1)
      return 'active'

    elif not self.beam_visible and not self.gate_visible:
      return 'lost_target'

    else:
      return 'aborted'
     
  def requestSpin(self, angleOffset):
    '''Requests a spin of angleOffset radians from controls'''
    setpoint = self.current_state + np.array([0, 0, 0, 0, 0, angleOffset, 0, 0, 0, 0, 0, 0])
    self.publishSetpoint(setpoint)
  
  def requestForwardMovement(self, distance):
    '''Requests a movement of distance meters forward relative to the robot's bearing'''
    setpoint = self.current_state + np.array([distance * np.cos(self.current_state[5]), distance * np.sin(self.current_state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    self.publishSetpoint(setpoint)

  def requestAnyMovement(self, direction, distance):
    if direction == "Forward":
      self.requestForwardMovement(distance)
    if direction == "Backward":
      self.requestForwardMovement(-1*distance)
    if direction == "Right":
      setpoint = self.current_state + np.array([distance * np.cos(self.current_state[5]+np.pi/2), distance * np.sin(self.current_state[5]+np.pi/2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
      self.publishSetpoint(setpoint)
    if direction == "Left":
      setpoint = self.current_state + np.array([distance * np.cos(self.current_state[5] - np.pi/2), distance * np.sin(self.current_state[5] - np.pi/2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
      self.publishSetpoint(setpoint)
  
  def publishSetpoint(self, setpoint):
    state = State()
    state.header = Header(self.header_seq, rospy.Time.now(), 'BASE')
    state.state = setpoint.tolist()
    self.header_seq += 1
    self.setpoint_publisher.publish(state)