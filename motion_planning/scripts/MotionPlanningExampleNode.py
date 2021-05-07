#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from MotionPlanningPublisher import MotionPlanningPublisher
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import dynamic_reconfigure.client

params = {}

def update(config):
    global params
    rospy.loginfo("""Config set to {waypoint_0_x}, {waypoint_0_y}, {waypoint_0_z}""".format(**config))
    params = config

def motionPlanningExampleNode():
    rospy.init_node('motion_planning_example_node', anonymous=True)
    motionPlanningPublisher = MotionPlanningPublisher()
    client = dynamic_reconfigure.client.Client("motion_planning_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'motion planning'

        # Create Path message
        trajectory = Path()
        trajectory.header = header

        waypoint0 = PoseStamped()
        waypoint0.header.stamp = rospy.Time.now()
        waypoint0.header.frame_id = 'waypoint0'
        waypoint0.pose.position = Point(params["warpoint_0_x"], params["warpoint_0_y"], params["warpoint_0_z"])
        waypoint0.pose.orientation = Quaternion(1, 0, -0.1, 0)

        trajectory.poses.append(waypoint0)

        waypoint1 = PoseStamped()
        waypoint1.header.stamp = rospy.Time.now() + rospy.Duration(10)
        waypoint1.header.frame_id = 'waypoint1'
        waypoint1.pose.position = Point(10, 2, -1)
        waypoint1.pose.orientation = Quaternion(1, 1, -0.1, 0.5)

        trajectory.poses.append(waypoint1)

        waypoint2 = PoseStamped()
        waypoint2.header.stamp = rospy.Time.now() + rospy.Duration(20)
        waypoint2.header.frame_id = 'waypoint2'
        waypoint2.pose.position = Point(10, 10, -2)
        waypoint2.pose.orientation = Quaternion(0, 1, -0.1, 0)

        trajectory.poses.append(waypoint2)

        # Publish messages
        motionPlanningPublisher.publishTrajectory(trajectory)
        rate.sleep()

if __name__ == '__main__':
    try:
        motionPlanningExampleNode()
    except rospy.ROSInterruptException:
        pass
