#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from controls.cfg import ControlsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {force_x}, {force_y}, {force_z}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("controls_server", anonymous = False)
    srv = Server(ControlsConfig, callback)
    rospy.spin()
