#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import navigation.msg

class Navigator():
  def __init__(self):
      rospy.init_node('state_machine')
      self.client = actionlib.SimpleActionClient('state_machine', navigation.msg.NavigationAction)
      #client.wait_for_server()
  
  def action(self, goal, measurement = None, motion_style = None):
      # Creates a goal to send to the action server.
      goal = navigation.msg.NavigationAction(goal=goal, measurement = measurement, motion_style = motion_style)

      # Sends the goal to the action server.
      self.client.send_goal(goal)

  def action_status(self, modified_mapping = None)
      # Prints out the result of executing the action
      mapping = {
        PENDIN:'active',
        ACTIVE:'active', 
        RECALLED:'abort', 
        REJECTED:'abort', 
        PREEMPTED:'', 
        ABORTED:'abort', 
        SUCCEEDED:'', 
        LOST:''
      }
      return self.client.get_state()

  def action_result(self)
      return self.client.get_result()