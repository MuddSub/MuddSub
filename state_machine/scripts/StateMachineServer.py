#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from state_machine.cfg import State_machineConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {shoulder_angle}, {hand_angle}, {release}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("state_machine_server", anonymous = False)
    srv = Server(State_machineConfig, callback)
    rospy.spin()
