#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg
from std_msgs.msg import Header
from vision.msg import Detection, DetectionArray
import tf2_ros
import numpy as np

class SimVisionPerfect:
    def __init__(self):
        self._visionPub = rospy.Publisher("vision/detections", DetectionArray, queue_size=5)

        self._modelStateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._statesCB)

        self._obstacles = set()

        ## TF2 transform broadcaster to update model state in TF
        self._tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Configure TF2 listener to get transforms from Gazebo
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

    def _statesCB(self, modelStates):
        """ callback for _modelStateSub, adds models to _obstacles """
        # NOTE: should find out what units the modelStates are in and what we need them to be
        # check for /robot_name

        # if model is visible, update the transform and add it to _obstacles

    def _updateTransform(self, name, pose):
        """ makes a transform message and publishes it using _tfBroadcaster """
        # create TransformStamped message

        # set header things

        # set transform data from pose

        # broadcast the transform

    def publishVision(self):
        """ create message for vision detections and publish it with _visionPub """
        # create detections list

        # iterate through list of _obstacles
        # look up transform between alfie and the obstacle
        # create detection and calculate detection info
        # add detection to detections list

        # make detections list into a message and publish it

def main():
    simVision = SimVision()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
      simVision.pubilshVision()
      rate.sleep()

if __name__ == '__main__':
    main()
