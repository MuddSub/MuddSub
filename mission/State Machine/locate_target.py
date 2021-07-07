#!/usr/bin/env python
import rospy
import smach
# import Map.msg      # whatever will import the correct slam messge

# very arbitrary constant values
threshold = .8
searchTime = 10
timeoutTime = 30

# userdata consists of different action takers 
# (navigators, map_reader) etc
# locate target keeps track of failures. return abort if > ACCEPTABLE_FAILURE
# cannot take longer than .5 seconds 
# (specifically, a time defined to be the required rxn time to respond to kill switch)
class LocateTarget(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target'])
    self.startTime = time()
    self.lastSearch = self.startTime
    # subscriber to be uncommented when we know the actual message type
    # self.slam_subscriber = rospy.Subscriber('slam/Map', geometry_msgs/PoseWithCovarianceStamped, self.callback, (userdata))
    self.target_confidence = 0

  def callback(self, data, userdata):
    for i in range(len(data)):
      if data[i][0] == userdata.target:
        self.target_confidence = data[i][4] # theoretically this is the confidence for the target position
        return data[i][4]
    self.target_confidence = 0
    return 0
  
  def execute(self, userdata):
    if self.target_confidence >= threshold: # slam's confidence in our target
      return 'success'
    else:
      if (time() - self.startTime) > timeoutTime:
        return 'abort'
      elif (time() - self.lastSearch) > searchTime:
        self.lastSearch = time()
        #initiate search pattern
    return 'active'
