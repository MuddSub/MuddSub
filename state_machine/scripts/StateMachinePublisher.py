#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalID
from state_machine.msg import Gripper
from state_machine.msg import Torpedo

class StateMachinePublisher:
    """
    Publishes state_machine messages describing the robot's desired objectives and the required gripper and torpedo commands.

    The robot's objectives, composed of a desired time and goal string, are represented by actionlib_msgs/GoalID messages published to the state_machine/objective topic.

    Gripper commands, composed of shoulder angle, hand angle, and release boolean, are represented by state_machine/Gripper messages published to the state_machine/gripper topic.

    Torpedo commands, composed of a release boolean and a vector direction, are represented by state_machine/Torpedo messages published to the state_machine/torpedo topic.

    Attributes:
        objective_pub: Publisher to state_machine/objective. Publishes actionlib_msgs/GoalID messages.
        gripper_pub: Publisher to state_machine/gripper. Publishes state_machine/Gripper messages.
        torpedo_pub: Publisher to state_machine/torpedo. Publishes state_machine/Torpedo messages.
    """

    def __init__(self):
        """Initialize StateMachinePublisher instance."""
        self.objective_pub = rospy.Publisher('state_machine/objective', GoalID, queue_size=10)
        self.gripper_pub = rospy.Publisher('state_machine/gripper', Gripper, queue_size=10)
        self.torpedo_pub = rospy.Publisher('state_machine/torpedo', Torpedo, queue_size=10)

    def publishObjective(self, objective):
        """
        Publish a actionlib_msgs/GoalID message.

        Uses the objective_pub Publisher to publish to state_machine/objective.

        Args:
            objective: The actionlib_msgs/GoalID message.
        """
        self.objective_pub.publish(objective)

    def publishGripperCommand(self, gripper_command):
        """
        Publish a state_machine/Gripper message.

        Uses the gripper_pub Publisher to publish to state_machine/gripper.

        Args:
            gripper_command: The state_machine/Gripper message.
        """
        self.gripper_pub.publish(gripper_command)

    def publishTorpedoCommand(self, torpedo_command):
        """
        Publish a state_machine/Torpedo message.

        Uses the torpedo_pub Publisher to publish to state_machine/torpedo.

        Args:
            torpedo_command: The state_machine/Torpedo message.
        """
        self.torpedo_pub.publish(torpedo_command)
