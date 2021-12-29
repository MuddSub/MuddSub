#!/usr/bin/env python3
import rospy
import smach
from slam.msg import Map, Obstacle
from std_msgs.msg import Bool

# check if we found in camera

'''
Case 1: we don't see the Gate in the camera, we need to look for it
     - Gate
       - Spin around first till we see it, then we can pass to "go to target"
     - For all other task
       - Look for the path finder and move accordingly, using the bottom facing camera
       - look going until we see the next Task in the camera/map
       - shift to "go to target"

Case 2: if we see the Gate, we move to "go to target"


background info we need to be passed in:
 1. the task_name

check if the task_name == Gate
 - spin around until we see the gate
  - What should we do if we still don't see the gate after 2 rotations?
     - potential solutions, move directly forward until we see the gate and pass to "go to target".

if the task_name != Gate
 - look for the path finder
  - move directly on top of it and then keep moving 
    so that the center of our center is lined up with the direction of the path finder
  - keep moving in the last direction until we see the task and pass to "go to target"
'''



class LocateTarget(smach.State):
  def __init__(self, task_name):
    rospy.loginfo("LocateTarget init")
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], 
                              input_keys = ['isWaiting_in'],
                              output_keys = ['isWaiting_out'])
    # self.locateTarget_subscriber = rospy.Subscriber('/mission/target', Bool, self.callback)
    self.found_target = False
    self.reached_requested_position = False
    self.spin_count = 0
    # self.startTime = rospy.get_time()
    # self.lastSearch = self.startTime
    self.task_name = task_name # we are not doing anything which this yet!
    rospy.loginfo("task_name is " + task_name)

  def callback(self, data):
    rospy.loginfo("The Data we recieve is " + str(data.data) + "...")
    if data.data:
        self.found = True
    self.lastSearch = rospy.get_time()
  
  def execute(self, userdata):
    if self.task_name == 'Gate':
      if self.found_target:
        return 'succeeded'

      elif userdata.isWaiting_in:
        if self.reached_requested_position:
          userdata.isWaiting_out = False
        return 'active'

      else:
        self.spin_count += 1
        if self.spin_count < 4:
          rospy.loginfo('Request to Spin')
        elif self.spin_count == 4:
          rospy.loginfo('Request to move 10 meters')
        elif self.spin_count >= 4:
          return 'abort'
        userdata.isWaiting_out = True
        return 'active'
  



