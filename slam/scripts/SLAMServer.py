#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from slam.cfg import SlamConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {position_x}, {position_y}, {position_z}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("slam_server", anonymous = False)
    srv = Server(SlamConfig, callback)
    rospy.spin()
