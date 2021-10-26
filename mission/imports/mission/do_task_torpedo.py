#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import Bool, Int32

# very arbitrary constant values
threshold = .8
timeoutTime = 30

# pwm values will be changing after testing
neutral_pwm = 1675
launch_pwm = 1225 

class DoTaskTorpedo(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target_hole'])
    self.startTime = time()
    # need to subscribe to some topic that will tell us if we're aligned properly with the torpedo, the way we do that can be changed
    self.torp_subscriber = rospy.Subscriber('/mechanisms/torpedo/alignment', Bool, self.callback, (userdata), queue_size=1)
    self.torp_publisher = rospy.Publisher('/mechanisms/torpedo/pwm', Int32, queue_size=10)
    self.aligned = False

  def callback(self, data):
    self.aligned = data

  def execute(self, userdata):
    if self.aligned:
      self.torp_publisher.publish(launch_pwm)
      return 'success'
    else:
      self.torp_publisher.publish(neutral_pwm)
      if (time() - self.startTime) > timeoutTime:
        return 'abort'
      else:
        # do the alignment
    return 'active'