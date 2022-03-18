#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import Bool
import mission.go_to_target as go_to_target
import mission.locate_target as locate_target
import mission.locate_target_tester as locate_target_tester
import mission.go_to_target_tester as go_to_target_tester
import mission.gate as gate
class MonitorKillSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active','aborted'])  
        self.to_kill = None
        self.kill_switch_subscriber = rospy.Subscriber('kill_switch',Bool, self.callback)

    def callback(self,data):
        self.to_kill = data.data
    def execute(self,data):
        if self.to_kill:
            return 'aborted'
        else: 
            return 'active'

def add_kill_monitor(State):
  registered_outcomes = list(State.get_registered_outcomes())
  print(registered_outcomes)
  for outcome in ['succeeded', 'aborted', 'active']:
    if outcome not in registered_outcomes:
      raise Exception("A monitored state must have the succeeded, aborted, and active outcomes")

  def monitor_task_child_cb(outcome):
      if outcome['MonitorKill']=='aborted' \
        or outcome['RunState']=='aborted' \
        or outcome['RunState']=='succeeded':
          return True
      else: 
          return False
  def monitor_task_outcome_cb(outcome):
      if outcome['MonitorKill']=='aborted': 
          return 'preempted'
      elif outcome['RunState']=='aborted':
          return 'aborted'
      elif outcome['RunState']=='succeeded':
          return 'succeeded'
      elif outcome['RunState']== 'active' \
        and outcome['MonitorKill']== 'active':
          return 'active'
      else:
          return 'aborted' 
  
  # REMAPPING = {"userdata":"userdata"}
#   MonitorState = smach.Concurrence(outcomes = ['preempted','aborted','succeeded','active'],
#                                       default_outcome= 'active',
#                                     child_termination_cb = monitor_task_child_cb,
#                                     outcome_cb = monitor_task_outcome_cb,
#                                     input_keys=['userdata'],output_keys=['userdata'])
  MonitorState = smach.Concurrence(outcomes = registered_outcomes,
                                    default_outcome = 'active',
                                    child_termination_cb = monitor_task_child_cb,
                                    outcome_cb = monitor_task_outcome_cb,
                                    input_keys=list(State.get_registered_input_keys()), output_keys=list(State.get_registered_output_keys()))

  with MonitorState:
    #   smach.Concurrence.add('RunState',State, remapping=REMAPPING)
      smach.Concurrence.add('RunState',State)
      smach.Concurrence.add('MonitorKill',MonitorKillSwitch())
  return MonitorState
  
def generate_task(task_name,taskAction):
  
  TaskUnderTemplate = smach.StateMachine(outcomes=['aborted','succeeded','preempted'],input_keys=['userdata'],output_keys=['userdata'])
  LocalizationPhrase = 'Locate'+task_name
  AdvancementPhrase = 'GoTo'+task_name
  TaskSpecificPhrase = 'ActOn'+task_name
  REMAPPING = {"userdata":"userdata"}
  TaskUnderTemplate.userdata.isWaiting = False

  with TaskUnderTemplate:
    print("1")
    # smach.StateMachine.add(LocalizationPhrase, add_kill_monitor(locate_target.LocateTarget(task_name)),
    #       transitions={'succeeded':AdvancementPhrase,
    #                    'active':  LocalizationPhrase,
    #                    'aborted':'aborted',
    #                    'preempted':'preempted'},
    #       remapping=REMAPPING)
    smach.StateMachine.add(LocalizationPhrase, add_kill_monitor(locate_target_tester.LocateTarget(task_name,'test_camera',0.8, [0.05,0.05,0.05,0.05])),
          transitions={'succeeded':AdvancementPhrase,
                       'active':  LocalizationPhrase,
                       'aborted':'aborted'},
        #   remapping={'locate_target_input': 'isWaiting',
        #              'locate_target_output': 'isWaiting'}
          remapping = {'isWaiting_in':'isWaiting',
                       'isWaiting_out':'isWaiting'})
    smach.StateMachine.add(AdvancementPhrase, add_kill_monitor(go_to_target_tester.GoToTarget(task_name,'test_camera')),
          transitions={'succeeded':TaskSpecificPhrase,
                       'active':AdvancementPhrase,
                       'aborted': 'aborted'},
          remapping=REMAPPING)
    smach.StateMachine.add(TaskSpecificPhrase,add_kill_monitor(gate.GateAction('test_camera',0.8, [0.05,0.05,0.05,0.05])), 
          transitions={'succeeded':'succeeded',
                       'active':TaskSpecificPhrase, 
                       'aborted':'aborted',
                       'lost_target':LocalizationPhrase},
          remapping=REMAPPING)
  return TaskUnderTemplate