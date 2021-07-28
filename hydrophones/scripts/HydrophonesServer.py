#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from hydrophones.cfg import HydrophonesConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {pinger_bearing}, {pinger_range_confidence},\
          {pinger_bearing_confidence}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("hydrophones_server", anonymous = False)
    srv = Server(HydrophonesConfig, callback)
    rospy.spin()
