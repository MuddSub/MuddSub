#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from std_msgs.msg import Int32
import numpy as np

def getKey():
    tty.setcbreak(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # print(key)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('thrusters_teleop')

    def set_speeds(speeds):
        rospy.set_param('drivers_server/pwm/thrusters/horizontal', {
            'bl': int(speeds[0, 0]),
            'br': int(speeds[0, 1]),
            'fl': int(speeds[0, 2]),
            'fr': int(speeds[0, 3])
        })
        rospy.set_param('drivers_server/pwm/thrusters/vertical', {
            'bl': int(speeds[1, 0]),
            'br': int(speeds[1, 1]),
            'fl': int(speeds[1, 2]),
            'fr': int(speeds[1, 3])
        })

    # hfl_publisher = rospy.Publisher('/robot/pwm/hfl', Int32, queue_size=1)
    # hfr_publisher = rospy.Publisher('/robot/pwm/hfr', Int32, queue_size=1)
    # hbl_publisher = rospy.Publisher('/robot/pwm/hbl', Int32, queue_size=1)
    # hbr_publisher = rospy.Publisher('/robot/pwm/hbr', Int32, queue_size=1)

    # vfl_publisher = rospy.Publisher('/robot/pwm/vfl', Int32, queue_size=1)
    # vfr_publisher = rospy.Publisher('/robot/pwm/vfr', Int32, queue_size=1)
    # vbl_publisher = rospy.Publisher('/robot/pwm/vbl', Int32, queue_size=1)
    # vbr_publisher = rospy.Publisher('/robot/pwm/vbr', Int32, queue_size=1)

    max_speed = 0.1  # Percentage of full speed
    forward = np.array([
        [1, 1, 1, 1],
        [0, 0, 0, 0]
    ])
    clockwise = np.array([
        [1, -1, 1, -1],
        [0, 0, 0, 0]
    ])
    up = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0]
    ])
    speeds = np.zeros([2, 4])

    while not rospy.is_shutdown():
        key = getKey()
        speeds = np.zeros([2, 4])
        if key == "w":
            speeds += forward
        elif key == "a":
            speeds -= clockwise
        elif key == "s":
            speeds -= forward
        elif key == "d":
            speeds += clockwise
        elif key == " ":
            speeds += up
        elif key == "z":
            speeds -= up

        speeds = np.clip(speeds, -max_speed, max_speed) * 1500 + 1500
        set_speeds(speeds)

    set_speeds(np.zeros([2, 4]))
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
