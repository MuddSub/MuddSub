#!/usr/bin/env python
import rospy
import tf2_geometry_msgs
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from nav_msgs.msg import Odometry
import sys
import tf2_ros
import copy

"""
- Subscribe to wrench messages and call Gazebo service
- Subscribe to gazebo pose and re-publish as /slam/robot/pose
"""
class GazeboInterface:
    def __init__(self):

        self.wrenchSub = rospy.Subscriber("/controls/robot/wrench", geometry_msgs.msg.WrenchStamped, self.wrenchCB)
        self.stateSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.stateCB)
        self.odomPub = rospy.Publisher("/slam/robot/pose", Odometry, queue_size=5)

        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
        except rospy.ROSException:
            rospy.logerr("Apply body wrench service not available. Closing node.")
            sys.exit(-1)
        self.wrench = geometry_msgs.msg.Wrench()

        try:
            self.applyWrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed, error=', e)
            sys.exit(-1)

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    # Do NED to ENU conversion
    def wrenchCB(self, wrench):

        try:
            trans = self.tfBuffer.lookup_transform("alfie/base_link_ned", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("whoops")
            return

        wrenchOut = geometry_msgs.msg.WrenchStamped()

        wrenchOut = tf2_geometry_msgs.do_transform_wrench(wrench, trans)

        self.wrench.force.x = wrenchOut.wrench.force.x
        self.wrench.force.y = wrenchOut.wrench.force.y
        self.wrench.force.z = wrenchOut.wrench.force.z

        self.wrench.torque.x = wrenchOut.wrench.torque.x
        self.wrench.torque.y = wrenchOut.wrench.torque.y
        self.wrench.torque.z = wrenchOut.wrench.torque.z


    # Get ENU frame from gazebo and re-publish as NED frame for MuddSub
    # Also, publish the TF2 transform in NED
    def stateCB(self, states):
        name = rospy.get_param('/robot_name')
        index = states.name.index(name)

        pose = states.pose[index]
        twist = states.twist[index]
        msg = Odometry()

        try:
            trans = self.tfBuffer.lookup_transform("world_ned", "world", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("whoops")
            return

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose

        msg.pose.pose = tf2_geometry_msgs.do_transform_pose(poseStamped, trans).pose

        linearStamped = geometry_msgs.msg.PointStamped()
        angluarStamped = geometry_msgs.msg.PointStamped()

        msg.twist.twist.linear = tf2_geometry_msgs.do_transform_point(linearStamped, trans).point
        msg.twist.twist.angular = tf2_geometry_msgs.do_transform_point(angluarStamped, trans).point

        self.odomPub.publish(msg)

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "alfie/base_link"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tfBroadcaster.sendTransform(t)


    def iterate(self, duration):
        bodyName = '%s/base_link' % rospy.get_param('/robot_name')

        success = self.applyWrench(
                    bodyName,
                    'world',
                    geometry_msgs.msg.Point(0,0,0),
                    self.wrench,
                    rospy.Time.now(),
                    rospy.Duration(duration))

        if not success:
            rospy.logerr("Body wrench apply service failed")

if __name__ == '__main__':
    rospy.init_node('gazebo_interface', anonymous=True)
    interface = GazeboInterface()

    updateRate = 20

    rate = rospy.Rate(updateRate)

    while not rospy.is_shutdown():
        interface.iterate(1./float(updateRate))
        rate.sleep()
