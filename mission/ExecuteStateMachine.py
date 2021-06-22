#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import Bool, Float64, String
import smach_ros
import sys

class MonitorKillSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','abort'])  
        self.to_kill = None
        self.kill_switch_subscriber = rospy.Subscriber('kill_switch',Bool, self.callback)

    def callback(self,data):
        self.to_kill = data.data
    def execute(self,data):
        print("updating kill switch")
        sys.stdout.flush()
        if self.to_kill:
            self.to_kill = None
            return 'abort'
        else: 
            return 'active'
class MonitorStartSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','abort','success'])  
        self.to_start = None
        self.start_switch_subscriber = rospy.Subscriber('start_switch',Bool, self.callback)
    def callback(self,data):
        self.to_start = data.data
    def execute(self,data):
        print("\nmonitor start "+str(self.to_start))
        sys.stdout.flush()
        if self.to_start==None:
            return 'active'
        elif self.to_start:
            self.to_start = None
            return 'success'
        else: 
            return 'abort'
class MonitorStartSwitchReset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','success'])  
        self.to_reset = None
        self.start_switch_subscriber = rospy.Subscriber('/start_switch',Bool, self.callback)
    def callback(self,data):
        self.to_start = data.data
    def execute(self,data):
        print("\nmonitor start reset "+str(self.to_start))
        sys.stdout.flush()
        if self.to_start==False:
            self.to_start = None
            return 'success'
        else:
            return 'active'
class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)      

    def execute(self, userdata):
        self.thrusterPub.publish(data=False)
        return 'success'


class RunMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['abort','success','active'])
        self.subscriber = rospy.Subscriber('test_mission',String, self.callback)
        self.status = 'waiting'
    def callback(self,data):
        self.to_start = data.data
    # call back seems weird
    def execute(self, userdata):
        #rospy.loginfo('Executing state RunMission with status '+self.status)
        print("\nmission: start sleep")
        sys.stdout.flush()
        for i in range(20):
          print(i,"seconds")
          sys.stdout.flush()
          rospy.sleep(1)
        print("mission: end sleep"+'\n')
        sys.stdout.flush()
        if self.status!=None and self.status in ['abort','success','active']:
            return self.status
        else:
            return 'active'
        

# main
def main():
    print("start")
    sys.stdout.flush()
    rospy.init_node('state_machine')
    def monitor_mission_child_cb(outcome):
        print("\nchild "+str(outcome)+'\n')
        sys.stdout.flush()
        '''
        if outcome['MonitorKillSwitch']=='abort' \
          or outcome['RunMission']=='abort' \
          or outcome['RunMission']=='success':
            return True
        else: 
            return False
        '''
        return True
    def monitor_mission_outcome_cb(outcome):
        print('\noutcome '+str(outcome)+'\n')
        sys.stdout.flush()
        if outcome['MonitorKillSwitch']=='abort' \
          or outcome['RunMission']=='abort':
            return 'abort'
        elif outcome['RunMission']=='success':
            return 'success'
        elif outcome['RunMission']== 'active' \
          and outcome['MonitorKillSwitch']== 'active':
            return 'active'
        else:
            return 'abort'
    
    MonitorMission = smach.Concurrence(outcomes = ['abort','success','active'],
                                        default_outcome= 'active',
                                      child_termination_cb = monitor_mission_child_cb,
                                      outcome_cb = monitor_mission_outcome_cb)
    with MonitorMission:
        smach.Concurrence.add('RunMission',RunMission())
        smach.Concurrence.add('MonitorKillSwitch',MonitorKillSwitch())

    BootingWorkflow = smach.StateMachine(outcomes=['abort','success'])

    with BootingWorkflow:
        smach.StateMachine.add('WaitToStart',MonitorStartSwitch(), 
                transitions={'success':'Running','active':'WaitToStart','abort':'Reset'})
        smach.StateMachine.add('Running',MonitorMission, 
                transitions={'success':'WaitToReset','abort':'Reset','active':'Running'})
        smach.StateMachine.add('WaitToReset', MonitorStartSwitchReset(), 
                transitions={'success':'Reset','active':'WaitToReset'})
        smach.StateMachine.add('Reset', Reset(), 
                transitions={'success':'WaitToStart','abort':'abort'})
    
    ready = False
    while not ready:
        ready = rospy.wait_for_message("/start", Bool)
    rospy.loginfo("Ready to boot")
    sis = smach_ros.IntrospectionServer('server_name', BootingWorkflow, '/SM_ROOT')
    sis.start()
    startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
    startPub.publish(data=False)	
    outcome = BootingWorkflow.execute()
    rospy.loginfo("Booted outcome "+outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()