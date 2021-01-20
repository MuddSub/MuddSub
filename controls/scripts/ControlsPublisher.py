#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from controls.msg import ThrusterForceArray
from controls.msg import ThrusterPWMArray

class ControlsPublisher:
    """
    Publishes controls messages describing the robot's desired motion and the required thruster commands.

    The target sum of forces and torques on the robot as a whole at any given point in time are described by geometry_msgs/WrenchStampes messages published to the controls/robot/wrench topic.

    The target force outputs of each thruster required to attain the target sum of forces and torques are described by controls/ThrusterForceArray messages published to the controls/thruster/forces topic.

    The pwm signals required to attain each thruster's force output target are described by controls/ThrusterPWMArray messages published to the controls/thruster/pwms topic.

    Attributes:
        wrench_pub: Publisher to controls/robot/wrench. Publishes geometry_msgs/WrenchStamped messages.
        forces_pub: Publisher to controls/thruster/forces. Publishes controls/ThrusterForceArray messages.
        pwms_pub: Publisher to controls/thruster/pwms. Publishes controls/ThrusterPWMArray messages.
    """

    def __init__(self):
        """Initialize ControlsPublisher instance."""
        self.wrench_pub = rospy.Publisher('controls/robot/wrench', WrenchStamped, queue_size=10)
        self.forces_pub = rospy.Publisher('controls/thruster/forces', ThrusterForceArray, queue_size=10)
        self.pwms_pub = rospy.Publisher('controls/thruster/pwms', ThrusterPWMArray, queue_size=10)

    def publishWrench(self, wrenchStamped):
        """
        Publish a geometry_msgs/WrenchStamped message.

        Uses the wrench_pub Publisher to publish to controls/robot/wrench.

        Args:
            wrenchStamped: The geometry_msgs/WrenchStamped message.
        """
        self.wrench_pub.publish(wrenchStamped)

    def publishForces(self, forces):
        """
        Publish a controls/ThrusterForceArray message.

        Uses the forces_pub Publisher to publish to controls/thruster/forces.

        Args:
            forces: The controls/ThrusterForceArray message.
        """
        self.forces_pub.publish(forces)

    def publishPWMs(self, pwms):
        """
        Publish a controls/ThrusterPWMArray message.

        Uses the pwms_pub Publisher to publish to controls/thruster/pwms.

        Args:
            pwms: The controls/ThrusterPWMArray message.
        """
        self.pwms_pub.publish(pwms)
