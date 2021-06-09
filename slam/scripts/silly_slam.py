#!/usr/bin/env python
import rospy
from drivers.msg import Depth
from sensor_msgs.msg import Imu
from SLAMPublisher import SLAMPublisher
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg

class SillySlam:

    def __init__(self):
        self.depthSub = rospy.Subscriber("/drivers/depth_sensor/depth", Depth, self.depthCB, queue_size=1)
        self.imuSub = rospy.Subscriber("/imu/imu", Imu, self.imuCB, queue_size=1)
        self.depth = 0
        self.slamPub = SLAMPublisher()
        self.frameBroadcaster = tf2_ros.TransformBroadcaster()
        self.depth = 0
        self.orientation = None

    def depthCB(self, msg):
        self.depth = msg.depth

    def imuCB(self, msg):
        self.orientation = msg.orientation

    def iterate(self):
        msg = Odometry()

        if self.orientation is None:
            msg.pose.pose.orientation.w = 1
        else:
            msg.pose.pose.orientation = self.orientation

        msg.pose.pose.position.z = self.depth
        self.slamPub.publishState(msg)

        t = geometry_msgs.msg.TransformStamped()

        t.header.frame_id = "world_ned"
        t.child_frame_id = "alfie/base_link_ned"

        t.transform.rotation = self.orientation
        t.transform.translation.z = self.depth

if __name__ == '__main__':
    rospy.init_node("silly_slam")
    ss = SillySlam()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ss.iterate()
        rate.sleep()
