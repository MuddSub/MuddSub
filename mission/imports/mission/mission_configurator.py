#!/usr/bin/env python3
import smach
import mission.mission_generator as mission_generator
import mission.gate as gate

def generate_mission():
  MissionRunner = smach.StateMachine(outcomes = ['aborted','succeeded','preempted'])
  MissionRunner.userdata.userdata = {}
  REMAPPING = {'userdata':'userdata'}  
  with MissionRunner:
    # for loop 
    smach.StateMachine.add("GateSequence",mission_generator.generate_task("Gate",\
      gate.GateAction('test_camera',0.8, [0.05,0.05,0.05,0.05])),\
      transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'},
      remapping=REMAPPING)
  return MissionRunner