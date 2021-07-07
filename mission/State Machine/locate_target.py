#!/usr/bin/env python
import rospy
import smach

# userdata consists of different action takers 
# (navigators, map_reader) etc
# locate target keeps track of failures. return abort if > ACCEPTABLE_FAILURE
# cannot take longer than .5 seconds 
# (specifically, a time defined to be the required rxn time to respond to kill switch)
class LocateTarget(smach.State):
  def __init__(self, task_name):
    pass
