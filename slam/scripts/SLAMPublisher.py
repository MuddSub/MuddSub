#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from slam.msg import Map

class SLAMPublisher:
    """
    Publishes SLAM messages describing the states of both the robot and the environment.

    The state of the robot, composed of its coordinates, velocity, and uncertainties are represented by nav_msgs/Odometry messages published to the slam/robot/state topic.

    The map of the environment, composed of landmark coordinates and uncertainties are represented by slam/Map messages published to the slam/map topic.

    Attributes:
        state_pub: Publisher to slam/robot/state. Publishes nav_msgs/Odometry messages.
        map_pub: Publisher to slam/map. Publishes slam/map messages.
    """

    def __init__(self):
        """Initialize SLAMPublisher instance."""
        self.state_pub = rospy.Publisher('slam/robot/state', Odometry, queue_size=10)
        self.map_pub = rospy.Publisher('slam/map', Map, queue_size=10)

    def publishState(self, state):
        """
        Publish a nav_msgs/Odometry message.

        Uses the state_pub Publisher to publish to slam/robot/state.

        Args:
            state: The nav_msgs/Odometry message.
        """
        self.state_pub.publish(state)

    def publishMap(self, map):
        """
        Publish a slam/Map message.

        Uses the map_pub Publisher to publish to slam/map.

        Args:
            map: The slam/Map message.
        """
        self.map_pub.publish(map)
