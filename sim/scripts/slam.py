#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from slam.msg import Map
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros

class SimSLAM:

  def __init__(self):

    self._mapPub = rospy.Publisher("slam/map", Map, queue_size=5)
    self._odomPub = rospy.Publisher("slam/robot/state", Odometry, queue_size=5)

    self._modelStateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._statesCB)

    self._seq = 0

    self._map = Map()
    self._odom = Odometry()

  def _statesCB(self, modelStates):
    if not rospy.has_param('/robot_name'):
      rospy.logwarn("Unable to get robot name from parameter server.")
      return
    robotName = rospy.get_param('/robot_name')

    self._map = Map()
    self._map.obstacle_poses = []


    try:
      trans = self._tfBuffer.lookup_transform("world_ned", "world", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr("Failed to get transform from World to world_ned")
      return 0

    for name,pose,twist in zip(modelStates.name, modelStates.pose, modelStates.twist):
      header = Header()
      header.frame_id = name
      header.seq = self._seq
      header.stamp = rospy.get_rostime()

      twist.linear.x, twist.linear.y = twist.linear.y, twist.linear.x
      twist.linear.z *= -1

      twist.angular.x, twist.angular.y = twist.angular.y, twist.angular.x
      twist.angular.z = -twist.angular.z


      poseStamped = geometry_msgs.msg.PoseStamped()
      poseStamped.pose = pose

      pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans).pose

      linearStamped = geometry_msgs.msg.PointStamped()
      angluarStamped = geometry_msgs.msg.PointStamped()

      if name == robotName:
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = name
        odom.pose.pose = pose
        odom.twist.twist = twist
        self._odom = odom

      else:
        obstaclePose = geometry_msgs.msg.PoseWithCovarianceStamped()
        obstaclePose.header = header
        obstaclePose.pose.pose = pose
        self._map.obstacle_poses.append(obstaclePose)

      self._seq += 1

  def publishSLAM(self):
    if self._seq == 0:
      rospy.logwarn("No SLAM to publish yet")
      return
    self._mapPub.publish(self._map)
    self._odomPub.publish(self._odom)


def main():
  simSlam = SimSLAM()

  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    simSlam.publishSLAM()
    rate.sleep()


if __name__ == '__main__':
  main()
