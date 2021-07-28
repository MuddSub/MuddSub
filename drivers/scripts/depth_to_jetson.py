#!/usr/bin/env python

import rospy
import serial
from DepthSensorPublisher import DepthSensorPublisher
from std_msgs.msg import Float64

global ser

def collectDepth():
	depth = ser.readline()
	depth = depth.strip()
	items = depth.split(',')
	if items[0] == "depth":
		return float(items[1])

if __name__ == '__main__':
	rospy.init_node('depth_sensor', anonymous=True)
	
	try:
		ser = serial.Serial('/dev/ttyACM0') # open serial port
	except: 
		rospy.logerr("Couldn't open serial")
		exit()
		
	if not ser.is_open:
		rospy.logerr("Couldn't open serial")
		exit()

	dsp = DepthSensorPublisher()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		depth = collectDepth()
		if depth is not None:
			dsp.publishDepth(collectDepth())
		rate.sleep()
