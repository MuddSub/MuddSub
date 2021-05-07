#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from ControlsPublisher import ControlsPublisher
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from controls.msg import ThrusterForceArray
from controls.msg import ThrusterPWMArray
import dynamic_reconfigure.client

def update(config):
    # Add whatever needs to be updated when configuration parameters change here
    rospy.loginfo("""Config set to {force_x}, {force_y}, {force_z}""".format(**config))

def controlsExampleNode():
    rospy.init_node('controls_example_node', anonymous=True)
    controlsPublisher = ControlsPublisher()
    client = dynamic_reconfigure.client.Client("controls_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        params = client.get_configuration()

        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'controls'

        # Create WrenchStamped message
        force = Vector3(params["force_x"], params["force_y"], params["force_z"])
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
