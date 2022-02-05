#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros

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
  
  def __init__(self):
    pass

  def execute(self, ud):
    rospy.loginfo("We are at gate.py")

