#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros
import mission.mission_configurator as mission_configurator

class StartSwitchMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','aborted','succeeded'])  
        self.to_start = None
        self.start_switch_subscriber = rospy.Subscriber('start_switch',Bool, self.callback)
    def callback(self,data):
        self.to_start = data.data
    def execute(self,data):
        if self.to_start==None:
            return 'active'
        elif self.to_start:
            return 'succeeded'
        else: 
            return 'aborted'

class ResetMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted','active'])
        self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)      
        self.to_reset = None
        self.start_switch_subscriber = rospy.Subscriber('/start_switch',Bool, self.callback)
    def callback(self,data):
        self.to_start = data.data
    def execute(self, userdata):
        self.thrusterPub.publish(data=False)
        if self.to_start:
            return 'active'
        else:
            return 'succeeded'


# main
def main():
    rospy.init_node('state_machine')
    MissionRunner = mission_configurator.generate_mission()
    Kernel = smach.StateMachine(outcomes=['aborted','succeeded'])

    with Kernel:
        smach.StateMachine.add('WaitToStart',StartSwitchMonitor(), 
                transitions={'succeeded':'Running','active':'WaitToStart','aborted':'Reset'})
        smach.StateMachine.add('Running', MissionRunner, 
                transitions={'succeeded':'WaitToReset','aborted':'Reset','preempted':'Reset'})
        smach.StateMachine.add('WaitToReset', ResetMonitor(), 
                transitions={'succeeded':'WaitToStart','active':'WaitToReset','aborted':'aborted'})
    
    ready = False
    while not ready:
        ready = rospy.wait_for_message("/start_state_machine", Bool)
    rospy.loginfo("Ready to boot")
    sis = smach_ros.IntrospectionServer('server_name', Kernel, '/SM_ROOT')
    sis.start()
    startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
    startPub.publish(data=False)	 # could move to state machine
    outcome = Kernel.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()