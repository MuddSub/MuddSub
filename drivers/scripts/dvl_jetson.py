#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32, Float64
from drivers.msg import DVL

if __name__ == '__main__':
	rospy.init_node('dvl_jetson', anonymous=True)
	try:
		ser = serial.Serial('/dev/ttyUSB1', timeout=0.1, baudrate=115200)
	except:
		rospy.logerr("Couldn't open serial")
		exit()
	dvlp = rospy.Publisher('/drivers/dvl', DVL, queue_size=1)
	rate = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		dvl_msg = DVL()
		sensor = ser.readline()
		print(sensor)
		sensor = sensor.strip()
		items = sensor.split(',')
		items = items[1:]
		if len(items) == 8:
			rospy.loginfo(sensor)
			dvl_msg.header.stamp = rospy.Time.now()
			dvl_msg.velocity.x = float(items[1])
			dvl_msg.velocity.y = float(items[2])
			dvl_msg.velocity.z = float(items[3])
			dvl_msg.fom = float(items[4])
			dvl_msg.altitude = float(items[5])
			dvl_msg.valid = items[6] == 'y'
			dvlp.publish(dvl_msg)
		rate.sleep()
