#!/usr/bin/env python3

import rospy
# import serial
from std_msgs.msg import Int32, Float64
from drivers.msg import DVL
import socket
import json

HOST = '192.168.194.95'
PORT = 16171

dvlsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
dvlsocket.connect((HOST, PORT))


if __name__ == '__main__':
	rospy.init_node('dvl_jetson', anonymous=True)
	# try:
	# 	ser = serial.Serial('/dev/ttyUSB0', timeout=0.1, baudrate=115200)
	# except:
	# 	rospy.logerr("Couldn't open serial")
	# 	exit()
	dvlp = rospy.Publisher('/drivers/dvl', DVL, queue_size=1)
	rate = rospy.Rate(100)

	
	socket_file = dvlsocket.makefile()
	while not rospy.is_shutdown():
		data = socket_file.readline()
		if len(data) == 0:
			break
		# rospy.loginfo(data)
		parsed_data = json.loads(data)
		
		dvl_msg = DVL()
		# print(parsed_data)
		# rospy.loginfo(parsed_data)
		dvl_msg.header.stamp = rospy.Time.now()
		dvl_msg.velocity.x = float(parsed_data.get('vx', 0))
		dvl_msg.velocity.y = float(parsed_data.get('vy', 0))
		dvl_msg.velocity.z = float(parsed_data.get('vz', 0))
		dvl_msg.fom = float(parsed_data.get('fom', 0))
		dvl_msg.altitude = float(parsed_data.get('altitude', 0))
		dvl_msg.valid = parsed_data.get('velocity_valid', False)
		dvlp.publish(dvl_msg)
		rate.sleep()
