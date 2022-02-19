#!/usr/bin/env python
from multiprocessing.dummy import active_children
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros
from vision.msg import DetectionArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from functools import reduce

class GateAction(smach.State):
  '''
  Move to Gate

  Once we detect the gate, we need to move(straight) to the gate (assuming there is no obstacles)
  We need to check our position relative to the gate as we move

  If we see the gate directly facing us, we move forward
    - Communicate with control?

  If we sdon't see the gate, we rotate and search for the gate again
    - Goes back to the previous state, locate target

  If we reached the gate, return success
    - how do we determine if we have reached the gate
    - How do we differentiate reaching gate and losing gate in sight

  ''' 
  
  def __init__(self,camera_name, min_confidence, thresholds):
    smach.State.__init__(self, outcomes=['active', 'success', 'abort','lost_target'],
                              input_keys = ['isWaiting_in',],
                              output_keys = ['isWaiting_out'])

    self.isVisible = rospy.Subscriber('vision/' + camera_name + '/detection_array', DetectionArray, self.gate_callback)
    self.reachedTarget_subscriber = rospy.Subscriber('/controls/robot/error', Odometry, self.reachedTarget_callback)
    
    self.camera_name = camera_name
    self.min_confidence = min_confidence
    self.threshold = thresholds

    self.gate_visible = True
    self.beam_visible = True
    self.reached_requested_pos = False

    self.range_to_beam = float('-inf')
    self.startTime = rospy.get_time()
    self.lastSearch = self.startTime


  def gate_callback(self,data):
    gate_flag = False
    beam_flag = False
    for i in data.detections:
      if i.name == 'gate' and i.confidence > self.min_confidence:
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
    return reduce(lambda x, y: x and y, map(lambda x: x < threshold, position))
  
  def execute(self, ud):
    if self.startTime - self.lastSearch > 60:
      return 'abort'

    elif abs(self.range_to_beam - 1) < 0.1:
      rospy.log('Ask to move to current position')
      rospy.log('Move to the right')
      rospy.log('Request to move 1 meter')
      return 'success'

    elif ud.isWaiting_in and (self.bem_visible or self.gate_visible):
      if self.reached_requested_pos:
          ud.isWaiting_out = False
      return 'active'

    elif not ud.isWaiting_in and (self.beam_visible or self.gate_visible):
      rospy.log("Request to Move Forward 1 meter")
      return 'active'

    elif not self.beam_visible and not self.gate_visible:
      return 'lost_target'

    else:
      return 'abort'
     
