#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import Bool
import template_task_generator

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
  
  REMAPPING = {"userdata":"userdata"}
  MonitorState = smach.Concurrence(outcomes = ['preempted','aborted','succeeded','active'],
                                      default_outcome= 'active',
                                    child_termination_cb = monitor_task_child_cb,
                                    outcome_cb = monitor_task_outcome_cb,
                                    input_keys=['userdata'],output_keys=['userdata'])

  with MonitorState:
      smach.Concurrence.add('RunState',State, remapping=REMAPPING)
      smach.Concurrence.add('MonitorKill',MonitorKillSwitch())
  return MonitorState
  
def generate_task(task_name,taskAction):
  
  TemplateTask = smach.StateMachine(outcomes=['aborted','succeeded','preempted'],input_keys=['userdata'],output_keys=['userdata'])
  locate_target = 'Locate'+task_name
  go_to_target = 'GoTo'+task_name
  act_on_task = 'ActOn'+task_name
  REMAPPING = {"userdata":"userdata"}
  with TemplateTask:
    smach.StateMachine.add(locate_target, add_kill_monitor(template_task_generator.LocateTarget(task_name)),
          transitions={'succeeded':go_to_target,'active':locate_target,'aborted':'aborted','preempted':'preempted'},
          remapping=REMAPPING)
    smach.StateMachine.add(go_to_target, add_kill_monitor(template_task_generator.GoToTarget(task_name)),
          transitions={'succeeded':act_on_task,'active':go_to_target,'aborted':locate_target,'preempted':'preempted'},
          remapping=REMAPPING)
    smach.StateMachine.add(act_on_task,add_kill_monitor(taskAction), 
          transitions={'succeeded':'succeeded','active':act_on_task, 'aborted':'aborted','preempted':'preempted'},
          remapping=REMAPPING)
  return TemplateTask