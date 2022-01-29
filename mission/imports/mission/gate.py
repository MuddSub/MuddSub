#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros

def gateLoateCB():
  pass

def gateGoToCB():
  pass

# class GateAction(smach.State):
#   def __init__(self):
#     smach.State.__init__(self, outcomes=[])

#   def execute(self, ud):
#     rospy.loginfo("We are at gate.py")



class GateAction(smach.State):
  def __init__(self):
    rospy.loginfo("LocateTarget init")
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], 
                              input_keys = ['isWaiting_in',],
                              output_keys = ['isWaiting_out'])
  def execute(self, userdata):
    return 'abort'
