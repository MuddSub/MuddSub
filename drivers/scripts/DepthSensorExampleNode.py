#!/usr/bin/env python

import rospy
from drivers.utils.DepthSensorPublisher import DepthSensorPublisher
from std_msgs.msg import Header
from drivers.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
import dynamic_reconfigure.client

def update(config):
    # Add whatever needs to be updated when configuration parameters change here
    rospy.loginfo("""Config set to {depth}""".format(**config))

def depthSensorExampleNode():
    rospy.init_node('depth_sensor_example_node', anonymous=True)
    depthSensorPublisher = DepthSensorPublisher()
    client = dynamic_reconfigure.client.Client("drivers_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        params = client.get_configuration()

        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'depth_sensor'

        # Create Depth message
        depth = Depth(header, params["depth"])

        # Create FluidPressure message
        pressure = FluidPressure(header, 3.0, 0.2)

        # Create Temperature message
        temperature = Temperature(header, 273.0, 30.2)

        # Publish messages
        depthSensorPublisher.publishDepth(depth)
        depthSensorPublisher.publishPressure(pressure)
        depthSensorPublisher.publishTemperature(temperature)
        rate.sleep()


if __name__ == '__main__':
    try:
        depthSensorExampleNode()
    except rospy.ROSInterruptException:
        pass
