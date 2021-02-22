#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path

class MotionPlanningPublisher:
    """
    Publishes trajectory messages describing the robot's desired trajectory.

    The robot's trajectory, composed of an array of positions, orientations, and times, is represented by nav_msgs/Path messages published to the motion_planning/trajectory topic.

    Attributes:
        _pathPub: Publisher to motion_planning/trajectory. Publishes nav_msgs/Path messages.
    """

    def __init__(self):
        """Initialize MotionPlanningPublisher instance."""
        self._pathPub = rospy.Publisher('motion_planning/trajectory', Path, queue_size=10)

    def publishTrajectory(self, trajectory):
        """
        Publish a nav_msgs/Path message.

        Uses the _pathPub Publisher to publish to motion_planning/trajectory.

        Args:
            trajectory: The nav_msgs/Path message.
        """
        self._pathPub.publish(trajectory)
