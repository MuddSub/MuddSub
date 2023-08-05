#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from drivers.cfg import DriversConfig
from drivers.cfg import ThrustersConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("drivers_server", anonymous = False)
    Server(DriversConfig, callback)
    Server(ThrustersConfig, callback, namespace="pwm/thrusters/horizontal")
    Server(ThrustersConfig, callback, namespace="pwm/thrusters/vertical")
    rospy.spin()
