#!/usr/bin/env python3
import rospy
import smach
from slam.msg import Map, Obstacle
from std_msgs.msg import Bool

# approach gate

class GoToTarget(smach.State):
  def __init__(self, task_name):
    rospy.loginfo("GoToTarget init")
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target', 'variance_threshold'])
    self.goToTarget_subscriber = rospy.Subscriber('/mission/reach_target', Bool, self.callback)
    self.reached = False
    self.startTime = rospy.get_time()
    self.lastSearch = self.startTime
    self.task_name = task_name # we are not doing anything which this yet!
  def callback(self, data):
    rospy.loginfo("The Data we recieve is " + data.data + "...")
    if data.data:
        self.reached = True
    self.lastSearch = rospy.get_time()
  
  def execute(self, userdata):
    if self.reached == False and self.startTime - self.lastSearch < 20:
        return 'active'
    elif self.reached == True:
        return 'succeeded'
    else:
        return 'abort'
