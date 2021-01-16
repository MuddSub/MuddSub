#!/usr/bin/env python

import rospy
from hydrophones.msg import PingerData

class HydrophonesPublisher:
    def __init__(self):
        self.hydrophones_data_pub = rospy.Publisher('hydrophones/data', PingerData, queue_size=10)

    def publishData(self, data):
        self.hydrophones_data_pub.publish(data)
