#!/usr/bin/env python

import rospy
from depth_sensor.msg import Depth
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature

class DepthSensorPublisher:
    def __init__(self):
        self.wrench_pub = rospy.Publisher('depth_sensor/depth', Depth, queue_size=10)
        self.pressure_pub = rospy.Publisher('depth_sensor/pressure', FluidPressure, queue_size=10)
        self.temperature_pub = rospy.Publisher('depth_sensor/temperature', Temperature, queue_size=10)

    def publishDepth(self, depth):
        self.wrench_pub.publish(depth)

    def publishPressure(self, pressure):
        self.pressure_pub.publish(pressure)

    def publishTemperature(self, temperature):
        self.temperature_pub.publish(temperature)
