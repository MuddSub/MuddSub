#!/usr/bin/env python

import rospy
from drivers.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature

class DepthSensorPublisher:
    """
    Publishes depth sensor data.

    The sensed depth of the robot is published in depth_sensor/Depth messages to the depth_sensor/depth topic.

    The sensed fluid pressure is published in sensor_msgs/FluidPressure messages to the depth_sensor/pressure topic.

    The sensed temperature is published in sensor_msgs/Temperature messages to the depth_sensor/temperature topic.

    Attributes:
        depth_pub: Publisher to depth_sensor/depth. Publishes drivers/Depth messages.
        pressure_pub: Publisher to depth_sensor/pressure. Publishes sensor_msgs/FluidPrssure messages.
        temperature_pub: Publisher to depth_sensor/temperature. Publishes sensor_msgs/Temperature messages.
    """

    def __init__(self):
        """Initialize DepthSensorPublisher instance."""
        self.depth_pub = rospy.Publisher('depth_sensor/depth', Depth, queue_size=10)
        self.pressure_pub = rospy.Publisher('depth_sensor/pressure', FluidPressure, queue_size=10)
        self.temperature_pub = rospy.Publisher('depth_sensor/temperature', Temperature, queue_size=10)

    def publishDepth(self, depth):
        """
        Publish a drivers/Depth message.

        Uses the depth_pub Publisher to publish to depth_sensor/depth.

        Args:
            depthStamped: The drivers/Depth message.
        """
        self.depth_pub.publish(depth)

    def publishPressure(self, pressure):
        """
        Publish a sensor_msgs/FluidPrssure message.

        Uses the pressure_pub Publisher to publish to depth_sensor/pressure.

        Args:
            pressure: The sensor_msgs/FluidPrssure message.
        """
        self.pressure_pub.publish(pressure)

    def publishTemperature(self, temperature):
        """
        Publish a sensor_msgs/Temperature message.

        Uses the temperature_pub Publisher to publish to depth_sensor/temperature.

        Args:
            temperature: The sensor_msgs/Temperature message.
        """
        self.temperature_pub.publish(temperature)
