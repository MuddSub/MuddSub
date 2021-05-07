#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from motion_planning.cfg import MotionPlanningConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {waypoint_0_x}, {waypoint_0_y}, {waypoint_0_z}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("motion_planning_server", anonymous = False)
    srv = Server(MotionPlanningConfig, callback)
    rospy.spin()
