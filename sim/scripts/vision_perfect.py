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

        # will be a set of obstacle names
        self._obstacles = set()

        ## TF2 transform broadcaster to update model state in TF
        self._tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Configure TF2 listener to get transforms from Gazebo
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

        # these numbers will depend on camera format
        # max possible with theia 125m lens
        # self._maxRange = 0 # not specified
        self._maxTheta = radians(135 / 2) # 135 degrees
        self._maxPhi  = radians(119 / 2) # 119 degrees


    def _statesCB(self, modelStates):
        """ callback for _modelStateSub, adds models to _obstacles """
        # TODO: check what units the modelStates are in and what we need them to be
        # check for /robot_name
        if not rospy.has_param('/robot_name'):
            rospy.logwarn("Unable to get robot name from parameter server.")
            return

        # update the transform and add the name of each obstacle to _obstacles
        for state in modelStates:
            if state.name == rospy.get_param('/robot_name'):
                continue
            self._updateTransform(state.name, state.pose)
            self._obstacles.add(state.name)


    def _updateTransform(self, name, pose):
        """ makes a transform message and publishes it using _tfBroadcaster """
        # create TransformStamped message
        tmsg = geometry_msgs.msg.TransformStamped()

        # set non transform things
        tmsg.header.stamp = rospy.time.now()
        tmsg.header.frame_id = "world"
        tmsg.child_frame_id = name

        # set transform data from pose
        tmsg.transform.translation.x = pose.position.x
        tmsg.transform.translation.y = pose.position.y
        tmsg.transform.translation.z = pose.position.z
        tmsg.transform.rotation.x = pose.orientation.x
        tmsg.transform.rotation.y = pose.orientation.y
        tmsg.transform.rotation.z = pose.orientation.z
        tmsg.transform.rotation.w = pose.orientation.w

        # broadcast the transform
        self._tfBroadcaster.sendTransform(tmsg)


    def publishVision(self):
        """ create message for vision detections and publish it with _visionPub """
        detections = []

        # iterate through list of _obstacles
        for obstacle in self._obstacles:
            # look up transform between alfie and the obstacle
            try:
                trans = self._tfBuffer.lookup_transform("alfie/base_link", obstacle, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to get transform from Alfie to the obstacle")
                return

            # create detection
            detection = Detection()
            detection.header.stamp = rospy.get_rostime()
            detection.header.frame_id = obstacle

            # calculate detection contents
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            dz = trans.transform.translation.z
            detection.range = np.sqrt(dx**2 + dy**2 + dz**2)
            detection.theta = np.arctan2(dx, dy)
            detection.phi = np.arcsin(dz/detection.range)
            detection.confidence = 1

            # check if detection would be visible by camera and add it to detection list
            if abs(detection.phi) <= self._maxPhi and abs(detection.theta) <= self._maxTheta:
                detections.append(detection)

        # make detections list into a message and publish it
        detectionsMessage = DetectionArray()
        detectionsMessage.header.stamp = rospy.get_rostime()
        detectionsMessage.detections = detections

        self._visionPub.publish(detectionsMessage)

def main():
    simVisionPerfect = SimVisionPerfect()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
      simVisionPerfect.pubilshVision()
      rate.sleep()

if __name__ == '__main__':
    main()
