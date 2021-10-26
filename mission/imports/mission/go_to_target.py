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

def main():
    rospy.init_node('go_to_target')

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

    # Create and start machine sm finished
    sis = smach_ros.IntrospectionServer('go_to_target_server', sm, 'SM_ROOT/TASK_LAYER/GO_TO_TARGET')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
