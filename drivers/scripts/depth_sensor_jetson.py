#!/usr/bin/env python

import rospy
from DepthSensorPublisher import DepthSensorPublisher
from std_msgs.msg import Header
from drivers.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from ms5837_python import *

def depthSensorJetson():
	rospy.init_node('depth_sensor_jetson', anonymous=True)
	dsp = DepthSensorPublisher()
	rate = rospy.Rate(10)
	setup() # setup function in get_depth

	while not rospy.is_shutdown():
		depth_val = get_depth_sensor()
		print(depth_val)
		dsp.publishDepth(depth_val)
		rate.sleep()

if __name__ == '__main__':
	try:
		depthSensorJetson()
	except rospy.ROSInterruptException:
		pass
