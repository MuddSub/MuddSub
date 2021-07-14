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
        _statePub: Publisher to slam/robot/state. Publishes nav_msgs/Odometry messages.
        _mapPub: Publisher to slam/map. Publishes slam/map messages.
    """

    def __init__(self):
        """Initialize SLAMPublisher instance."""
        self._statePub = rospy.Publisher('slam/robot/state', Odometry, queue_size=10)
        self._mapPub = rospy.Publisher('slam/map', Map, queue_size=10)

    def publishState(self, state):
        """
        Publish a nav_msgs/Odometry message.
        Uses the _statePub Publisher to publish to slam/robot/state.
        Args:
            state: The nav_msgs/Odometry message.
        """
        self._statePub.publish(state)

    def publishMap(self, map):
        """
        Publish a slam/Map message.
        Uses the _mapPub Publisher to publish to slam/map.
        Args:
            map: The slam/Map message.
        """
        self._mapPub.publish(map)