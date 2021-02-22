#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from HydrophonesPublisher import HydrophonesPublisher
from hydrophones.msg import PingerData
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3

def hydrophonesExampleNode():
    rospy.init_node('hydrophones_example_node', anonymous=True)
    hydrophonesPublisher = HydrophonesPublisher()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'hydrophones'

        # Create PingerData message
        pinger_range = Range()
        pinger_range.radiation_type = 0 #it says uint8, but not sure how this relates to sound
        pinger_range.field_of_view = 1.0
        pinger_range.min_range = 1.0
        pinger_range.max_range = 2.0
        pinger_range.range = 1.5

        pinger_bearing = 1.56
        pinger_range_confidence = 0.7
        pinger_bearing_confidence = 0.8

        pinger_data = PingerData(header, pinger_range, pinger_bearing, pinger_range_confidence, pinger_bearing_confidence)
        hydrophonesPublisher.publishData(pinger_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        hydrophonesExampleNode()
    except rospy.ROSInterruptException:
        pass
