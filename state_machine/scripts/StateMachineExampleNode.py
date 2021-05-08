#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from StateMachinePublisher import StateMachinePublisher
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Vector3
from state_machine.msg import Gripper
from state_machine.msg import Torpedo
import dynamic_reconfigure.client

def update(config):
    #Add whatever needs to be updated when configuration parameters change here
    rospy.loginfo("""Reconfigure Request: {shoulder_angle}, {hand_angle}, {release}""".format(**config))


def stateMachineExampleNode():
    rospy.init_node('state_machine_example_node', anonymous=True)
    stateMachinePublisher = StateMachinePublisher()
    client = dynamic_reconfigure.client.Client("state_machine_server", timeout=30, config_callback=update)
    client.update_configuration({})

    rate = rospy.Rate(1)


    while not rospy.is_shutdown():
        params = client.get_configuration(timeout=10)
        print("this is params ", params)
        # Create Header message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'state machine'

        # Create Objective message
        objective = GoalID()
        objective.stamp = rospy.Time.now()
        objective.id = 'test'

        # Create Gripper message
        #shoulder_angle = 1
        #hand_angle = 0.5
        #release = False
        shoulder_angle = params["shoulder_angle"]
        hand_angle = params["hand_angle"]
        release = params["release"]
        gripper_command = Gripper(header, shoulder_angle, hand_angle, release)
        print("shoulder_angle ", shoulder_angle)
        print("hand_angle ", hand_angle)
        print("gripper release ", release)
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
