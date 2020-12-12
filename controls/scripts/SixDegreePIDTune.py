#!/usr/bin/env python

import rospy


if __name__ == '__main__':
    rospy.init_node("six_degree_pid_tune")

    rospy.set_param("surge/kP", 0)
    rospy.set_param("sway/kP", 0)
    rospy.set_param("roll/kP", 0)
    rospy.set_param("pitch/kP", 0)
    rospy.set_param("yaw/kP", 0)

    rospy.set_param("surge/kI", 0)
    rospy.set_param("sway/kI", 0)
    rospy.set_param("roll/kI", 0)
    rospy.set_param("pitch/kI", 0)
    rospy.set_param("yaw/kI", 0)

    rospy.set_param("surge/kD", 0)
    rospy.set_param("sway/kD", 0)
    rospy.set_param("roll/kD", 0)
    rospy.set_param("pitch/kD", 0)
    rospy.set_param("yaw/kD", 0)

    rospy.set_param("heave/kP", -5)
    rospy.set_param("heave/kI", -4)
    rospy.set_param("heave/kD", -.05)
