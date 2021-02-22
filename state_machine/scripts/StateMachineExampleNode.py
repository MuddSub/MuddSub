#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from StateMachinePublisher import StateMachinePublisher
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Vector3
from state_machine.msg import Gripper
from state_machine.msg import Torpedo

def stateMachineExampleNode():
    rospy.init_node('state_machine_example_node', anonymous=True)
    stateMachinePublisher = StateMachinePublisher()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'state machine'

        # Create Objective message
        objective = GoalID()
        objective.stamp = rospy.Time.now()
        objective.id = 'test'

        # Create Gripper message
        shoulder_angle = 1
        hand_angle = 0.5
        release = False
        gripper_command = Gripper(header, shoulder_angle, hand_angle, release)

        # Create Torpedo message
        release = True
        direction = Vector3(1, 1, 0)
        torpedo_command = Torpedo(header, release, direction)

        # Publish messages
        stateMachinePublisher.publishObjective(objective)
        stateMachinePublisher.publishGripperCommand(gripper_command)
        stateMachinePublisher.publishTorpedoCommand(torpedo_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        stateMachineExampleNode()
    except rospy.ROSInterruptException:
        pass
