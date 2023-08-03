#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def main():
    hfl_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    hfr_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    hbl_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    hbr_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    vfl_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    vfr_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    vbl_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    vbr_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)

    rospy.init_node('pwm_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 20Hz
    while not rospy.is_shutdown():
        horizontal_thrusters = rospy.get_param('drivers_server/pwm/thrusters/horizontal', {'bl': 1500, 'br': 1500, 'fl': 1500, 'fr': 1500})
        vertical_thrusters = rospy.get_param('drivers_server/pwm/thrusters/vertical', {'bl': 1500, 'br': 1500, 'fl': 1500, 'fr': 1500})
        
        hfl_publisher.publish(Int32(horizontal_thrusters['fl']))
        hfr_publisher.publish(Int32(horizontal_thrusters['fr']))
        hbl_publisher.publish(Int32(horizontal_thrusters['bl']))
        hbr_publisher.publish(Int32(horizontal_thrusters['br']))

        vfl_publisher.publish(Int32(vertical_thrusters['fl']))
        vfr_publisher.publish(Int32(vertical_thrusters['fr']))
        vbl_publisher.publish(Int32(vertical_thrusters['bl']))
        vbr_publisher.publish(Int32(vertical_thrusters['br']))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass