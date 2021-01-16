#!/usr/bin/env python

import rospy
from HydrophonesPublisher import HydrophonesPublisher

if __name__ == '__main__':
    try:
        rospy.init_node('hydrophonesPublisher', anonymous=True)
        hydrophonesPublisher = HydrophonesPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
