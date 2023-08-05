#!/usr/bin/env python3
import rospy
import smach
from slam.msg import Map, Obstacle
from std_msgs.msg import Bool

from vision.msg import DetectionArray
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# from functools import reduce
# approach gate

class GoToTarget(smach.State):
  def __init__(self, task_name,camera_name):
    rospy.loginfo("GoToTarget init")
    smach.State.__init__(self, outcomes=['active', 'succeeded', 'aborted'], input_keys = ['target', 'variance_threshold'])
    self.goToTarget_subscriber = rospy.Subscriber('/mission/reach_target', Bool, self.callback)
    self.detection_subscriber = rospy.Subscriber('vision/' + camera_name + '/detection_array', DetectionArray, self.detection_callback)
    self.reached = False
    self.startTime = rospy.get_time()
    self.lastSearch = self.startTime
    self.centered = False
    self.task_name = task_name # we are not doing anything which this yet!
  
  def detection_callback(self, data):
    for i in data.detections:
      if i.name == 'gate' and i.confidence > self.min_confidence:
        self.found_target = True
        self.centered = abs(i.boundingBox.center.x - 0.5) < 0.01
        break

  
  def callback(self, data):
    rospy.loginfo("The Data we recieve is " + data.data + "...")
    if data.data:
        self.reached = True
    self.lastSearch = rospy.get_time()
  
  def execute(self, userdata):
    return 'succeeded'
    # if self.reached == False and self.startTime - self.lastSearch < 20:
    #     return 'active'
    # elif self.reached == True:
    #     return 'succeeded'
    # else:
    #     return 'aborted'
