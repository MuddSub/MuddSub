#!/usr/bin/env python
from multiprocessing.dummy import active_children
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
    smach.State.__init__(self, outcomes=['active', 'success', 'abort','lost_target'],
                              input_keys = ['isWaiting_in',],
                              output_keys = ['isWaiting_out'])
    self.visible = True
    self.reached_requested_pos = False
    self.success = False

  def execute(self, ud):
    if self.success:
      return 'success'
    
    elif ud.isWaiting_in and self.visible:
      if self.reached_requested_pos:
          ud.isWaiting_out = False
      return 'active'

    elif not ud.isWaiting_in and self.visible:
      rospy.log("Request to Move Forward 1 meter")
      return 'active'

    elif not self.visible:
      #Determine whether right in front of gate or lost gate
      return 'success'

    else:
      return 'lost_target'

