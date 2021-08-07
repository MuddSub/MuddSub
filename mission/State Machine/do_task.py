#!/usr/bin/env python
import rospy
import smach
from slam.msg import Map, Obstacle

# TODO: construct task state machine for this file

class DoTask(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['gate', 'buoy', 'gripper', 'torpedo', 'surface', 'abort'], input_keys = ['task'])
  
  def execute(self, userdata):
    if userdata.task is in ['gate', 'buoy', 'gripper', 'torpedo', 'surface']:
      return userdata.task
    return 'abort'
