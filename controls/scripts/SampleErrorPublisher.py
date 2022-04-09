#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance, Twist, Point, Quaternion, Vector3
import numpy as np
from tf.transformations import quaternion_from_euler

def publishOdometry():
    pub = rospy.Publisher('/controls/robot/error',Odometry, queue_size=10)
    rospy.init_node('SampleOdometry',anonymous=True)
    rate = rospy.Rate(10)

    orientation = quaternion_from_euler(0,0,0)

    pose = Pose()
    pose.position = Point(0,0,0)
    pose.orientation = Quaternion(orientation[1],orientation[2],orientation[3],orientation[0])

    odometry = Odometry()
    odometry.pose = PoseWithCovariance(pose,[0.0]*36)

    #
    linear = Vector3(0.0,0.0,0.0)
    angular = Vector3(0.0,0.0,0.0)

    twist = Twist(linear,angular)
    odometry.twist = TwistWithCovariance(twist,[0.0]*36)

    odometry.header = Header()
    odometry.child_frame_id = ""

    pub.publish(odometry)
    print("Published odometry")

if __name__ == '__main__':
    while not rospy.is_shutdown():
        publishOdometry()
        #rospy.spin()