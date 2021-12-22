#!/usr/bin/env python3
import rospy
import smach
from slam.msg import Map, Obstacle
from std_msgs.msg import Bool

# check if we found in camera

class LocateTarget(smach.State):
  def __init__(self, task_name):
    rospy.loginfo("LocateTarget init")
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target', 'variance_threshold'])
    self.locateTarget_subscriber = rospy.Subscriber('/mission/target', Bool, self.callback)
    self.found = False
    self.startTime = rospy.get_time()
    self.lastSearch = self.startTime
    self.task_name = task_name # we are not doing anything which this yet!
  def callback(self, data):
    rospy.loginfo("The Data we recieve is " + str(data.data) + "...")
    if data.data:
        self.found = True
    self.lastSearch = rospy.get_time()
  
  def execute(self, userdata):
    if self.found == False:
        return 'active'
    elif self.found == True:
        return 'succeeded'
    else:
        return 'abort'
