#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16


class Server:
    def __init__(self):
        self.orientation = None
        self.velocity = None

    def orientation_callback(self, msg):
        # "Store" message received.
        self.orientation = msg

    def velocity_callback(self, msg):
        # "Store" the message received.
        self.velocity = msg

    def print_data(self):
        print("orientation: ", self.orientation)
        print("velocity: ", self.velocity)

if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/orientation', Int16 , server.orientation_callback)
    rospy.Subscriber('/velocity', Int16, server.velocity_callback)
    while True:
        server.print_data()
    rospy.spin()
