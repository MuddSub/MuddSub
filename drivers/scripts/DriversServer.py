#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from drivers.cfg import DriversConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {depth}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("drivers_server", anonymous = False)
    srv = Server(DriversConfig, callback)
    rospy.spin()
