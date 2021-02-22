#include "state_machine/StateMachinePublisher.hh"

namespace MuddSub::StateMachine
{
  StateMachinePublisher::StateMachinePublisher(ros::NodeHandle n):
    n_(n)
  {
    objectivePub_ = n_.advertise<actionlib_msgs::GoalID>("state_machine/objective", 1000);
    gripperPub_ = n_.advertise<state_machine::Gripper>("state_machine/gripper", 1000);
    torpedoPub_ = n_.advertise<state_machine::Torpedo>("state_machine/torpedo", 1000);
  }

  void StateMachinePublisher::publishObjective(actionlib_msgs::GoalID& objective)
  {
    objectivePub_.publish(objective);
  }

  void StateMachinePublisher::publishGripperCommand(state_machine::Gripper& gripperCommand)
  {
    gripperPub_.publish(gripperCommand);
  }

  void StateMachinePublisher::publishTorpedoCommand(state_machine::Torpedo& torpedoCommand)
  {
    torpedoPub_.publish(torpedoCommand);
  }
}
