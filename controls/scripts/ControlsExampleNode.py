#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from ControlsPublisher import ControlsPublisher
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from controls.msg import ThrusterForceArray
from controls.msg import ThrusterPWMArray

def controlsExampleNode():
    rospy.init_node('controls_example_node', anonymous=True)
    controlsPublisher = ControlsPublisher()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'controls'

        # Create WrenchStamped message
        force = Vector3(1.0, 2.0, 3.0)
        torque = Vector3(4.0, 5.0, 6.0)
        wrench = WrenchStamped(header, Wrench(force, torque))

        # Create ThrusterForceArray message
        forces = ThrusterForceArray(header, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])

        # Create ThrusterPWMArraymessage
        pwms = ThrusterPWMArray(header, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])

        # Publish messages
        controlsPublisher.publishWrench(wrench)
        controlsPublisher.publishForces(forces)
        controlsPublisher.publishPWMs(pwms)
        rate.sleep()

if __name__ == '__main__':
    try:
        controlsExampleNode()
    except rospy.ROSInterruptException:
        pass
