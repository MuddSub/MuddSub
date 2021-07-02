#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32, Float64

from DepthSensorPublisher import DepthSensorPublisher
from drivers.msg import DVL

global ser
thrusters = [1500]*8

def collectSensors():
	sensor = ser.readline()
	sensor = sensor.strip()
	items = sensor.split(',')
	rospy.loginfo(sensor)
	if items[0] == "depth" and len(items) == 2:
		return "depth", float(items[1])
	elif len(items) == 8:
		return "dvl", items

def pulseToSerial(msg, i):
		pulse = msg.data
		rospy.loginfo(pulse)
		thrusters[i] = pulse

if __name__ == '__main__':
		

		rospy.init_node('teensy_interface', anonymous=True)
		
		try:
			ser = serial.Serial('/dev/ttyACM0', timeout=0.1) # open serial port
		except: 
			rospy.logerr("Couldn't open serial")
			exit()
			
		if not ser.is_open:
			rospy.logerr("Couldn't open serial")
			exit()

		hfl_subscriber = rospy.Subscriber('/robot/pwm/hfl', Int32, pulseToSerial, (0), queue_size=1)
		hfr_subscriber = rospy.Subscriber('/robot/pwm/hfr', Int32, pulseToSerial, (1), queue_size=1)
		hbl_subscriber = rospy.Subscriber('/robot/pwm/hbl', Int32, pulseToSerial, (2), queue_size=1)
		hbr_subscriber = rospy.Subscriber('/robot/pwm/hbr', Int32, pulseToSerial, (3), queue_size=1)

		vfl_subscriber = rospy.Subscriber('/robot/pwm/vfl', Int32, pulseToSerial, (4), queue_size=1)
		vfr_subscriber = rospy.Subscriber('/robot/pwm/vfr', Int32, pulseToSerial, (5), queue_size=1)
		vbl_subscriber = rospy.Subscriber('/robot/pwm/vbl', Int32, pulseToSerial, (6), queue_size=1)
		vbr_subscriber = rospy.Subscriber('/robot/pwm/vbr', Int32, pulseToSerial, (7), queue_size=1)

	
		dsp = DepthSensorPublisher()
		dvlp = rospy.Publisher('/drivers/dvl', DVL, queue_size=1)
		rate = rospy.Rate(10000)
		while not rospy.is_shutdown():
			# sensor = collectSensors()
			# if sensor is not None and sensor[0] == "depth":
			# 	dsp.publishDepth(sensor[1])
			# elif sensor is not None and sensor[0] == "dvl":
			# 	dvl_msg = DVL()
			# 	sensor[1][0] = rospy.Time.now()
				# dvl_msg.header.stamp = rospy.Time.from_sec(float(sensor[1][0]))
				# dvl_msg.header.stamp = sensor[1][0]
				# dvl_msg.velocity.x = float(sensor[1][1])
				# dvl_msg.velocity.y = float(sensor[1][2])
				# dvl_msg.velocity.z = float(sensor[1][3])
				# dvl_msg.fom = float(sensor[1][4])
				# dvl_msg.altitude = float(sensor[1][5])
				# dvl_msg.valid = sensor[1][6] == 'y'
				# dvl_msg.status = bool(float(sensor[1][7]))
				# dvlp.publish(dvl_msg)
			ser.write("thrust,0{},1{},2{},3{},4{},5{},6{},7{}\n".format(*thrusters))	
			rate.sleep()
			

