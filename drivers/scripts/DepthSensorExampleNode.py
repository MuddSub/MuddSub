#!/usr/bin/env python

import rospy
from DepthSensorPublisher import DepthSensorPublisher
from std_msgs.msg import Header
from drivers.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature

def depthSensorExampleNode():
    rospy.init_node('depth_sensor_example_node', anonymous=True)
    depthSensorPublisher = DepthSensorPublisher()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'depth_sensor'

        # Create Depth message
        depth = Depth(header, 10.0)

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
