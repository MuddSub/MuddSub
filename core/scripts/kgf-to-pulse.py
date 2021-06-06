#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32


def pulseWidth(pulse):
    vfl_publisher = rospy.Publisher('vfl_pwm', Int32, queue_size=10)
    vfr_publisher = rospy.Publisher('vfr_pwm', Int32, queue_size=10)
    vbl_publisher = rospy.Publisher('vbl_pwm', Int32, queue_size=10)
    vbr_publisher = rospy.Publisher('vbr_pwm', Int32, queue_size=10)
    hfl_publisher = rospy.Publisher('hfl_pwm', Int32, queue_size=10)
    hfr_publisher = rospy.Publisher('hfr_pwm', Int32, queue_size=10)
    hbl_publisher = rospy.Publisher('hbl_pwm', Int32, queue_size=10)
    hbr_publisher = rospy.Publisher('hbr_pwm', Int32, queue_size=10)
    rospy.loginfo(pulse)
    pub.publish(pulse)

def kgfToPulse(msg):
    kgf = msg.data
    if kgf < -0.01:
        pulse = 210.336 * kgf + 1471.912
    elif kgf > 0.01:
        pulse = 147.464 * kgf + 1528.659
    else:
        pulse = 1500
    pulseWidth(int(pulse))

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    vfl_subscriber = rospy.Subscriber('vfl_forces', Float64, kgfToPulse)
    vfr_subscriber = rospy.Subscriber('vfr_forces', Float64, kgfToPulse)
    vbl_subscriber = rospy.Subscriber('vbl_forces', Float64, kgfToPulse)
    vbr_subscriber = rospy.Subscriber('vbr_forces', Float64, kgfToPulse)
    hfl_subscriber = rospy.Subscriber('hfl_forces', Float64, kgfToPulse)
    hfr_subscriber = rospy.Subscriber('hfr_forces', Float64, kgfToPulse)
    hbl_subscriber = rospy.Subscriber('hbl_forces', Float64, kgfToPulse)
    hbr_subscriber = rospy.Subscriber('hbr_forces', Float64, kgfToPulse)
    rospy.spin()

