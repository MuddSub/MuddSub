#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from slam.msg import Map

class SLAMPublisher:
    def __init__(self):
        self.wrench_pub = rospy.Publisher('slam/robot/state', Odometry, queue_size=10)
        self.map_pub = rospy.Publisher('slam/map', Map, queue_size=10)

    def publishState(self, state):
        self.wrench_pub.publish(state)

    def publishMap(self, map):
        self.map_pub.publish(map)
