#!/usr/bin/env python

import rospy
from ControlsPublisher import ControlsPublisher

if __name__ == '__main__':
    try:
        rospy.init_node('ControlsPubNode', anonymous=True)
        controlsPublisher = ControlsPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
