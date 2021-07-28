import rospy
import smach
import smach_ros
from time import time
import actionlib
import navigation.msg


class Navigate(smach.State):
  def __init__(self, goal, measurement, motion_style, timeout):
    smach.State.__init__(self,
                          outcomes = ['success','active','abort'],
                          input_keys = ['navigator'])
    
    self.initialized = False
    self.timeout = timeout

    self.clinet = None
    self.goal = navigation.msg.NavigationAction(goal=goal, measurement = measurement, motion_style = motion_style)

  def execute(self, userdata):
    navigator = userdata.navigator

    # use action server here?
    if not self.initialized:
      navigator.send_goal(self.goal)
      self.initialized = True
      self.startTime = time()

    else:

      action_status = self.client.get_state()
      # Possible States Are: PENDING, ACTIVE, RECALLED,
      # REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

      if action_status == SUCCEEDED:
        return 'success'

      elif time()-self.startTime <= self.timeout and action_status in [ACTIVE, LOST]:
        if action_status == LOST:
          # to do: log message lost
          navigator.send_goal(self.goal)
          return 'active'

        else: return 'abort'

      else: return 'abort'


      '''
      if abs(self.heave.plantState - self.heave.setpoint) < .025:
        self.successCount += 1
      else:
        self.successCount = 0
      # use time instead of succesCount?


      if successCount > self.targetSuccessCount:
        return 'success'
      elif time-self.startTime > userdata.timeout:
        return 'abort'
      else:
        return 'active'
      '''
  # sample template?

  # create goTo template
  # we can template this as goTo template, and add a parallel detection scheme (paralle scheme not necessary if slam works.)

  # interface with navigation: action lib server library, use time to control? navigation says i develop a position?

  # interface with slam. if slam is not working, navigation also not work.
