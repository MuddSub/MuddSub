#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from SLAMPublisher import SLAMPublisher
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from slam.msg import Map

def slamExampleNode():
    rospy.init_node('slam_example_node', anonymous=True)
    slamPublisher = SLAMPublisher()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'slam'

        # Create Odometry message
        pos = Point(0.0, 1.0, 2.0)
        rot = Quaternion(0.4, 0.2, 0.3, 0.5)
        pose = PoseWithCovariance(Pose(pos, rot), [0.8] * 36)
        lin = Vector3(4.0, 2.0, 5.0)
        ang = Vector3(0.4, 0.8, 0.2)
        twist = TwistWithCovariance(Twist(lin, ang), [0.2] * 36)
        state = Odometry(header, '', pose, twist)

        # Create Map message
        landmarkHeader = Header()
        landmarkHeader.stamp = rospy.Time.now()
        landmarkHeader.frame_id = 'gate'
        map = Map(header, [PoseWithCovarianceStamped(landmarkHeader, pose)])

        # Publish messages
        slamPublisher.publishState(state)
        slamPublisher.publishMap(map)
        rate.sleep()

if __name__ == '__main__':
    try:
        slamExampleNode()
    except rospy.ROSInterruptException:
        pass
