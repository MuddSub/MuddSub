#!/usr/bin/env python3

import rospy
from drivers.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature

class DepthSensorPublisher:
    """
    Publishes depth sensor data.

    The sensed depth of the robot is published in drivers/depth_sensor/Depth messages to the drivers/depth_sensor/depth topic.

    The sensed fluid pressure is published in sensor_msgs/FluidPressure messages to the drivers/depth_sensor/pressure topic.

    The sensed temperature is published in sensor_msgs/Temperature messages to the drivers/depth_sensor/temperature topic.

    Attributes:
        _depthPub: Publisher to drivers/depth_sensor/depth. Publishes drivers/Depth messages.
        _pressurePub: Publisher to drivers/depth_sensor/pressure. Publishes sensor_msgs/FluidPrssure messages.
        _temperaturePub: Publisher to drivers/depth_sensor/temperature. Publishes sensor_msgs/Temperature messages.
    """

    def __init__(self):
        """Initialize DepthSensorPublisher instance."""
        self._depthPub = rospy.Publisher('drivers/depth_sensor/depth', Depth, queue_size=10)
        self._pressurePub = rospy.Publisher('drivers/depth_sensor/pressure', FluidPressure, queue_size=10)
        self._temperaturePub = rospy.Publisher('drivers/depth_sensor/temperature', Temperature, queue_size=10)

    def publishDepth(self, depth):
        """
        Publish a drivers/Depth message.

        Uses the _depthPub Publisher to publish to drivers/depth_sensor/depth.

        Args:
            depthStamped: The drivers/Depth message.
        """
        self._depthPub.publish(depth)

    def publishPressure(self, pressure):
        """
        Publish a sensor_msgs/FluidPrssure message.

        Uses the _pressurePub Publisher to publish to drivers/depth_sensor/pressure.

        Args:
            pressure: The sensor_msgs/FluidPrssure message.
        """
        self._pressurePub.publish(pressure)

    def publishTemperature(self, temperature):
        """
        Publish a sensor_msgs/Temperature message.

        Uses the _temperaturePub Publisher to publish to drivers/depth_sensor/temperature.

        Args:
            temperature: The sensor_msgs/Temperature message.
        """
        self._temperaturePub.publish(temperature)
