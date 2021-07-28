#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg
from std_msgs.msg import Header
from vision.msg import Detection, DetectionArray
import tf2_ros
import numpy as np

class SimVision:

  def __init__(self):
    self._visionPub = rospy.Publisher("vision/detections", DetectionArray, queue_size=5)

    self._modelStateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._statesCB)

    self._obstacles = set()

    self._seq = 0

    ## TF2 transform broadcaster to update model state in TF
    self._tfBroadcaster = tf2_ros.TransformBroadcaster()

    # Configure TF2 listener to get transforms from Gazebo
    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

  def _statesCB(self, modelStates):


    if not rospy.has_param('/robot_name'):
      rospy.logwarn("Unable to get robot name from parameter server.")
      return
    robotName = rospy.get_param('/robot_name')

    index = modelStates.name.index(robotName)

    for name,pose,twist in zip(modelStates.name, modelStates.pose, modelStates.twist):
      if name == robotName:
        continue

      self._updateTransform(name, pose)
      self._obstacles.add(name)


  def _updateTransform(self, name, pose):
    t = geometry_msgs.msg.TransformStamped()

    # Also update the transform
    time = rospy.get_time()

    t.header.stamp = rospy.Time.now()

    t.header.frame_id = "world"
    t.child_frame_id = name
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z

    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w

    self._tfBroadcaster.sendTransform(t)

  def publishVision(self):
    detections = []

    for obstacle in self._obstacles:
      try:
        trans = self._tfBuffer.lookup_transform("alfie/base_link", obstacle, rospy.Time())
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get transform from Alfie to the obstacle")
        return 0

      dx = trans.transform.translation.x
      dy = trans.transform.translation.y
      dz = trans.transform.translation.z

      range = np.sqrt(dx**2 + dy**2 + dz**2)
      theta = np.arctan2(dx, dy)
      phi = np.arcsin(dz/range)

      detection = Detection()

      detection.header.stamp = rospy.get_rostime()
      detection.header.seq = self._seq
      detection.header.frame_id = obstacle

      detection.range = range
      detection.phi = phi
      detection.theta = theta
      detection.confidence = 1

      detections.append(detection)

    detectionsMessage = DetectionArray()
    detectionsMessage.header.stamp = rospy.get_rostime()
    detectionsMessage.header.seq = self._seq

    detectionsMessage.detections = detections

    self._visionPub.publish(detectionsMessage)

def main():
  simVision = SimVision()

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    simVision.pubilshVision()
    rate.sleep()


if __name__ == '__main__':
  main()
