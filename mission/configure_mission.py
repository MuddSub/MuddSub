#!/usr/bin/env python
import smach
import mission_generator
import gate

def generate_mission():
  Mission = smach.StateMachine(outcomes = ['aborted','succeeded','preempted'])
  Mission.userdata.userdata = {}
  REMAPPING = {'userdata':'userdata'}  
  with Mission:
    # for loop 
    smach.StateMachine.add("GateSequence",mission_generator.generate_task("Gate",\
      gate.GateAction()),\
      transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'},
      remapping=REMAPPING)
  return Mission 