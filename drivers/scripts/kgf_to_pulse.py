#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32

def pulseWidth(pulse, publisher):
    rospy.loginfo(pulse)
    publisher.publish(pulse)

def kgfToPulse(msg, publisher):
    kgf = msg.data
    if kgf < -0.01:
        pulse = 210.336 * kgf + 1471.912
    elif kgf > 0.01:
        pulse = 147.464 * kgf + 1528.659
    else:
        pulse = 1500
    if pulse > 1900:
        pulse = 1900
    elif pulse < 1100:
        pulse = 1100
    pulseWidth(int(pulse), publisher)

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    vfl_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    vfr_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    vbl_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    vbr_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)
    hfl_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    hfr_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    hbl_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    hbr_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    vfl_subscriber = rospy.Subscriber('/controls/thruster_forces/vfl', Float64, kgfToPulse, (vfl_publisher), queue_size=1)
    vfr_subscriber = rospy.Subscriber('/controls/thruster_forces/vfr', Float64, kgfToPulse, (vfr_publisher), queue_size=1)
    vbl_subscriber = rospy.Subscriber('/controls/thruster_forces/vbl', Float64, kgfToPulse, (vbl_publisher), queue_size=1)
    vbr_subscriber = rospy.Subscriber('/controls/thruster_forces/vbr', Float64, kgfToPulse, (vbr_publisher), queue_size=1)
    hfl_subscriber = rospy.Subscriber('/controls/thruster_forces/hfl', Float64, kgfToPulse, (hfl_publisher), queue_size=1)
    hfr_subscriber = rospy.Subscriber('/controls/thruster_forces/hfr', Float64, kgfToPulse, (hfr_publisher), queue_size=1)
    hbl_subscriber = rospy.Subscriber('/controls/thruster_forces/hbl', Float64, kgfToPulse, (hbl_publisher), queue_size=1)
    hbr_subscriber = rospy.Subscriber('/controls/thruster_forces/hbr', Float64, kgfToPulse, (hbr_publisher), queue_size=1)

    rospy.spin()

