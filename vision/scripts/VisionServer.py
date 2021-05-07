#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from vision.cfg import VisionConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {example_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("vision_server", anonymous = False)
    srv = Server(VisionConfig, callback)
    rospy.spin()
