#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from controls.msg import ThrusterForceArray
from controls.msg import ThrusterPWMArray

class ControlsPublisher:
    def __init__(self):
        self.wrench_pub = rospy.Publisher('controls/robot/wrench', WrenchStamped, queue_size=10)
        self.forces_pub = rospy.Publisher('controls/thruster/forces', ThrusterForceArray, queue_size=10)
        self.pwms_pub = rospy.Publisher('controls/thruster/pwms', ThrusterPWMArray, queue_size=10)

    def publishWrench(self, wrenchStamped):
        self.wrench_pub.publish(wrenchStamped)

    def publishForces(self, forces):
        self.forces_pub.publish(forces)

    def publishPWMs(self, pwms):
        self.pwms_pub.publish(pwms)
