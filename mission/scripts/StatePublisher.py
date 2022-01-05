#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion, Vector3
import numpy as np
from tf.transformations import quaternion_from_euler

# Row-major representation of the 6x6 covariance matrix

# The orientation parameters use a fixed-axis representation.

# In order, the parameters are:

# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)

seq = 0

def getMessage():
    global seq

    position = rospy.get_param('mission_server/spoof/slam/pose/position', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    rot = rospy.get_param('mission_server/spoof/slam/pose/orientation', {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    linVel = rospy.get_param('mission_server/spoof/slam/twist/linear', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    angVel = rospy.get_param('mission_server/spoof/slam/twist/angular', {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})

    # Create the PoseWithCovariance
    position = Point(position['x'], position['y'], position['z']) #x, y, z
    orientation = Quaternion(*quaternion_from_euler(rot['roll'], rot['pitch'], rot['yaw'])) #x, y, z, w
    pose = Pose(position, orientation)
    cov = np.diag([1] * 6).reshape(-1).tolist()
    poseWithCov = PoseWithCovariance(pose, cov)

    # Create the TwistWithCovariance
    linearVel = Vector3(linVel['x'], linVel['y'], linVel['z'])
    angularVel = Vector3(angVel['roll'], angVel['pitch'], angVel['yaw'])
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