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
    pulseWidth(int(pulse), publisher)

if __name__ == '__main__':
    rospy.init_node('force_to_pwm', anonymous=True)
    vfl_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=10)
    vfr_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=10)
    vbl_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=10)
    vbr_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=10)
    hfl_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=10)
    hfr_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=10)
    hbl_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=10)
    hbr_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=10)
    vfl_subscriber = rospy.Subscriber('/controls/thruster_forces/vfl', Float64, kgfToPulse, (vfl_publisher))
    vfr_subscriber = rospy.Subscriber('/controls/thruster_forces/vfr', Float64, kgfToPulse, (vfr_publisher))
    vbl_subscriber = rospy.Subscriber('/controls/thruster_forces/vbl', Float64, kgfToPulse, (vbl_publisher))
    vbr_subscriber = rospy.Subscriber('/controls/thruster_forces/vbr', Float64, kgfToPulse, (vbr_publisher))
    hfl_subscriber = rospy.Subscriber('/controls/thruster_forces/hfl', Float64, kgfToPulse, (hfl_publisher))
    hfr_subscriber = rospy.Subscriber('/controls/thruster_forces/hfr', Float64, kgfToPulse, (hfr_publisher))
    hbl_subscriber = rospy.Subscriber('/controls/thruster_forces/hbl', Float64, kgfToPulse, (hbl_publisher))
    hbr_subscriber = rospy.Subscriber('/controls/thruster_forces/hbr', Float64, kgfToPulse, (hbr_publisher))
    rospy.spin()

