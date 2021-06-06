#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32

def pulseToSerial(msg):
    pulse = msg.data
    ser = serial.Serial('/dev/ttyUSB0') # open serial port
    ser.write(f"thrust {pulse} \n")
    ser.close()

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    subscriber = rospy.Subscriber('pwm', Int32, pulseToSerial)
    rospy.spin()

