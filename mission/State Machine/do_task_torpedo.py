#!/usr/bin/env python
import rospy
import smach

# very arbitrary constant values
threshold = .8
alignTime = 10
timeoutTime = 30

class DoTaskTorpedo(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target_hole'])
    self.startTime = time()
    # need to subscribe to some topic that will tell us if we're aligned properly with the torpedo
    # self.torp_subscriber = rospy.Subscriber('torpedo topic', /robot/torpedo, self.callback, (userdata))
    self.alignment_confidence = 0
    self.lastAlign = self.startTime

  def callback(self, data):
    # should update self.alignment_confidence
    pass

  def launch(self):
    # move the servo that will launch the torpedo
    pass

  def execute(self, userdata):
    if self.alignment_confidence >= threshold:
      self.launch()
      return 'success'     # don't think that we can actually know if we succeeded, but there aren't torpedo redos as far as I'm aware
    else:
      if (time() - self.startTime) > timeoutTime:
        return 'abort'
      elif (time() - self.lastAlign) > alignTime:
        self.lastAlign = time()
        # do the alignment
    return 'active'