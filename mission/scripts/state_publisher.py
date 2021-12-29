#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion, Vector3
import numpy as np

# Row-major representation of the 6x6 covariance matrix

# The orientation parameters use a fixed-axis representation.

# In order, the parameters are:

# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)

seq = 0

def getMessage():
    global seq
    # building PoseWithCovariance msg
    # This represents a pose in free space with uncertainty.

    # Create the PoseWithCovariance
    x = rospy.get_param("/spoof/state/pose/x", 0.0)
    position = Point(x, 0.0, 0.0) #x, y, z
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0) #x, y, z, w
    pose = Pose(position, orientation)
    cov = np.diag([1] * 6).reshape(-1).tolist()
    poseWithCov = PoseWithCovariance(pose, cov)

    # Create the TwistWithCovariance
    linearVel = Vector3(0.0, 0.0, 0.0)
    angularVel = Vector3(0.0, 0.0, 0.0)
    twist = Twist(linearVel, angularVel)
    twistWithCov = TwistWithCovariance(twist, cov)

    # Create the header
    header = Header(seq, rospy.Time.now(), 'BASE')
    seq += 1

    # Finally, create the odometry message
    odometry = Odometry()
    odometry.header = header
    odometry.pose = poseWithCov
    odometry.twist = twistWithCov
    return odometry

def main():
    pub = rospy.Publisher('/slam/robot/state', Odometry, queue_size=10)
    rospy.init_node('state_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        pub.publish(getMessage())
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass