#!/usr/bin/env python
import rospy
import smach
from slam.msg import Map, Obstacle

# very arbitrary constant values
searchTime = 10
timeoutTime = 30

# userdata consists of different action takers 
# (navigators, map_reader) etc
# locate target keeps track of failures. return abort if > ACCEPTABLE_FAILURE
# cannot take longer than .5 seconds 
# (specifically, a time defined to be the required rxn time to respond to kill switch)
class LocateTarget(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['active', 'success', 'abort'], input_keys = ['target', 'variance_threshold'])
    self.startTime = time()
    self.lastSearch = self.startTime
    self.slam_subscriber = rospy.Subscriber('/slam/map', Map, self.callback, (userdata))
    self.target_confidence = None

  def callback(self, data, userdata):
    for i in range(len(data)):
      if data[i][1] == userdata.target: # the obstacle at index i's name will be in data[i][1]
        # getting the variance values from the diagonals of the covariance matrix
        self.target_confidence[0] = data[i][3][0]
        self.target_confidence[1] = data[i][3][4]
        self.target_confidence[2] = data[i][3][8]
        return self.target_confidence
    self.target_confidence = None # if we don't get entries of data that match our desired target, set the confidence to None
    return self.target_confidence

  def search(self, userdata):
    # need to decide on what search pattern to do
    pass
  
  def execute(self, userdata):
    if self.target_confidence is not None and self.target_confidence[0] <= userdata.variance_threshold and self.target_confidence[1] <= userdata.variance_threshold and self.target_confidence[2] <= userdata.variance_threshold:
      return 'success'
    else:
      if (time() - self.startTime) > timeoutTime:
        return 'abort'
      elif (time() - self.lastSearch) > searchTime:
        self.lastSearch = time()
        #initiate search pattern
        self.search(userdata)
    return 'active'
