#!/usr/bin/env python

import rospy
from VisionPublisher import VisionPublisher

if __name__ == '__main__':
    try:
        rospy.init_node('VisionPubNode', anonymous=True)
        visionPublisher = VisionPublisher("test_camera")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
