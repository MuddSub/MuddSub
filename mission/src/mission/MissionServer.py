#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from mission.cfg import PositionConfig, OrientationConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {position_x}, {position_y}, {position_z}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("mission_server", anonymous = False)

    # The top level namespace "mission_server" automatically gets appended
    Server(PositionConfig, callback, namespace="spoof/slam/pose/position")
    Server(OrientationConfig, callback, namespace="spoof/slam/pose/orientation")
    Server(PositionConfig, callback, namespace="spoof/slam/twist/linear")
    Server(OrientationConfig, callback, namespace="spoof/slam/twist/angular")
    Server(PositionConfig, callback, namespace="spoof/setpoint/pose/position")
    Server(OrientationConfig, callback, namespace="spoof/setpoint/pose/orientation")
    Server(PositionConfig, callback, namespace="spoof/setpoint/twist/linear")
    Server(OrientationConfig, callback, namespace="spoof/setpoint/twist/angular")
    rospy.spin()
