#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import Bool, String
import smach_ros
import configure_mission

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
class StartSwitchResetMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','succeeded'])  
        self.to_reset = None
        self.start_switch_subscriber = rospy.Subscriber('/start_switch',Bool, self.callback)
    def callback(self,data):
        self.to_start = data.data
    def execute(self,data):
        if self.to_start or self.to_start==None:
            return 'active'
        else:
            return 'succeeded'
class ResetMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)      

    def execute(self, userdata):
        self.thrusterPub.publish(data=False)
        return 'succeeded'


class RunMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted','succeeded','active'])
        self.subscriber = rospy.Subscriber('test_mission',String, self.callback)
        self.status = 'waiting'
    def callback(self,data):
        self.to_start = data.data
    # call back seems weird
    def execute(self, userdata):
        rospy.loginfo('Executing state RunMission with status '+self.status)
        if self.status!=None and self.status in ['aborted','succeeded','active']:
            return self.status
        else:
            return 'active'
        

# main
def main():
    rospy.init_node('state_machine')
    
    Kernel = smach.StateMachine(outcomes=['aborted','succeeded'])

    with Kernel:
        smach.StateMachine.add('WaitToStart',StartSwitchMonitor(), 
                transitions={'succeeded':'Running','active':'WaitToStart','aborted':'Reset'})
        smach.StateMachine.add('Running', configure_mission.generate_mission(), 
                transitions={'succeeded':'WaitToReset','aborted':'Reset','preempted':'Reset'})
        smach.StateMachine.add('WaitToReset', StartSwitchResetMonitor(), 
                transitions={'succeeded':'Reset','active':'WaitToReset'})
        smach.StateMachine.add('Reset', ResetMonitor(), 
                transitions={'succeeded':'WaitToStart','aborted':'aborted'})
    
    ready = False
    while not ready:
        ready = rospy.wait_for_message("/start_state_machine", Bool)
    rospy.loginfo("Ready to boot")
    sis = smach_ros.IntrospectionServer('server_name', Kernel, '/SM_ROOT')
    sis.start()
    startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
    startPub.publish(data=False)	
    outcome = Kernel.execute()
    rospy.loginfo("Booted outcome "+outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()