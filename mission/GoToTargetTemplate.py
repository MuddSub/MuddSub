import rospy
import smach
import smach_ros 
from time import time 
import actionlib
import navigation.msg
class SLAMInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','abort'])  
        self.slam_string = None
        self.slam_map = None
        self.slam_subscriber = rospy.Subscriber('slam',String, self.callback)
    def stringToMap(self, s):
        array = s.split(';')
        # gate/x:10,y:10,confidence:2; 
        return {item.split('/')[0]:{subitem.split(':')[0]:subitem.split(':')[1] for subitem in item.split('/')[1]} for item in array}

    def callback(self,data):
        self.slam_map = self.stringToMap(data.data)
    def execute(self,data):
        if self.to_kill:
            return 'abort'
        else: 
            return 'active'
    
class Navigate(smach.State):
  def __init__(self, goal, measurement, motion_style, timeout):
    smach.State.__init__(self,
                          outcomes = ['success','active','abort'],
                          input_keys = ['navigator'])
    
    self.initialized = False
    self.timeout = timeout
    
    self.clinet = actionlib.SimpleActionClient('')
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


threshold = .8
searchTime = 10
timeoutTime = 30

class locateTarget(smach.State):
  
  def __init__(self):

    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target'])
    self.startTime = time()
    self.lastSearch = self.startTime

  def execute(self, userdata):

    if slamConfidence >= threshold: #slam's confidence in our target

      return 'success'

    else:

      if (time() - self.startTime) > timeoutTime:

        return 'abort'

      elif (time() - self.lastSearch) > searchTime:
        
        self.lastSearch = time()
        #initiate search pattern

    return 'active'



class goToTarget(smach.State):

  def __init__(self):

    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target'])
    self.startTime = time()
    self.goal = navigation.msg.NavigationAction(goal=goal, measurement = measurement, motion_style = motion_style) #edit to do this with input keys
    navigator.send_goal(self.goal)
    

  def execute(self, userdata):

    if (time() - self.startTime) > timeoutTime:

      return 'abort'

    if #success message from navigation

      return 'success'

    return 'active'


class performAction(smach.State):

  def __init__(self):

  def execute(self, userdata):