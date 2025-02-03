#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from drivers.msg import DVL
import math

if __name__ == "__main__":
    rospy.init_node('dvl_debugging', anonymous=True)

    dvlp = rospy.Publisher('/drivers/dvl_debug', DVL, queue_size=1)
    rate = rospy.Rate(100)
    # Test path is given by (cos t, sin t, 0.3t)
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        dvl_msg = DVL()
        time = rospy.get_time()
        dvl_msg.header.stamp = rospy.Time.now()
        dvl_msg.velocity.x = -math.sin(time-start_time)
        dvl_msg.velocity.y = math.cos(time-start_time)
        dvl_msg.velocity.z = 0.3
        dvl_msg.fom = 0.0
        dvl_msg.altitude = 0.0
        dvl_msg.valid = True
        rospy.loginfo(f"x: {math.cos(time-start_time)}, y: {math.sin(time-start_time)}, z: {0.3*(time-start_time)}")
        dvlp.publish(dvl_msg)
        rate.sleep()