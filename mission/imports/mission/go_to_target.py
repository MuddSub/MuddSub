#!/usr/bin/env python3

import rospy
import smach
import smach_ros

# define state CheckMap
class GetTargetLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'succeeded'

# define state GenerateTrajectory
class GenerateTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'succeeded'

# ExecuteTrajectory
class ExecuteTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed', 'map_changed'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'map_changed'

def GoToTarget(task_name):
    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['target_reached', 'target_not_found', 'generate_trajectory_failed', 'execute_trajectory_failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GET_TARGET_LOCATION', GetTargetLocation(),
            transitions={
                'succeeded': 'GENERATE_TRAJECTORY',
                'failed': 'target_not_found'
            },
            remapping={}
        )
        smach.StateMachine.add('GENERATE_TRAJECTORY', GenerateTrajectory(),
            transitions={
                'succeeded': 'EXECUTE_TRAJECTORY',
                'failed': 'generate_trajectory_failed'
            },
            remapping={}
        )
        smach.StateMachine.add('EXECUTE_TRAJECTORY', ExecuteTrajectory(),
            transitions={
                'succeeded': 'target_reached',
                'failed': 'execute_trajectory_failed',
                'map_changed': 'GET_TARGET_LOCATION'
            },
            remapping={}
        )
    
    return sm
