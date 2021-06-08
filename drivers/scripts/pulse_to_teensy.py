#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32

def pulseToSerial(msg, i):
    pulse = msg.data
    rospy.loginfo(pulse)
    ser = serial.Serial('/dev/ttyACM0') # open serial port
    ser.write("thrust,{}{}\n".format(i, pulse))
    ser.close()

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    vfl_subscriber = rospy.Subscriber('/robot/pwm/vfl', Int32, pulseToSerial, (5))
    vfr_subscriber = rospy.Subscriber('/robot/pwm/vfr', Int32, pulseToSerial, (2))
    vbl_subscriber = rospy.Subscriber('/robot/pwm/vbl', Int32, pulseToSerial, (5))
    vbr_subscriber = rospy.Subscriber('/robot/pwm/vbr', Int32, pulseToSerial, (2))
    hfl_subscriber = rospy.Subscriber('/robot/pwm/hfl', Int32, pulseToSerial, (4))
    hfr_subscriber = rospy.Subscriber('/robot/pwm/hfr', Int32, pulseToSerial, (0))
    hbl_subscriber = rospy.Subscriber('/robot/pwm/hbl', Int32, pulseToSerial, (7))
    hbr_subscriber = rospy.Subscriber('/robot/pwm/hbr', Int32, pulseToSerial, (3))
    rospy.spin()

