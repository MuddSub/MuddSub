#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32

def pulseToSerial(msg):
    pulse = msg.data
    rospy.loginfo(pulse)
    ser = serial.Serial('/dev/ttyACM0') # open serial port
    ser.write("thrust,4{}\n".format(pulse))
    ser.close()

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    vfl_subscriber = rospy.Subscriber('/robot/pwm/vfl', Int32, pulseToSerial)
    vfr_subscriber = rospy.Subscriber('/robot/pwm/vfr', Int32, pulseToSerial)
    vbl_subscriber = rospy.Subscriber('/robot/pwm/vbl', Int32, pulseToSerial)
    vbr_subscriber = rospy.Subscriber('/robot/pwm/vbr', Int32, pulseToSerial)
    hfl_subscriber = rospy.Subscriber('/robot/pwm/hfl', Int32, pulseToSerial)
    hfr_subscriber = rospy.Subscriber('/robot/pwm/hfr', Int32, pulseToSerial)
    hbl_subscriber = rospy.Subscriber('/robot/pwm/hbl', Int32, pulseToSerial)
    hbr_subscriber = rospy.Subscriber('/robot/pwm/hbr', Int32, pulseToSerial)
    rospy.spin()

