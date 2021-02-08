#include "state_machine/StateMachinePublisher.hh"

namespace MuddSub::StateMachine
{
  StateMachinePublisher::StateMachinePublisher(ros::NodeHandle n):
    n_(n)
  {
    objective_pub = n_.advertise<actionlib_msgs::GoalID>("state_machine/objective", 1000);
    gripper_pub = n_.advertise<state_machine::Gripper>("state_machine/gripper", 1000);
    torpedo_pub = n_.advertise<state_machine::Torpedo>("state_machine/torpedo", 1000);
  }

  void StateMachinePublisher::publishObjective(actionlib_msgs::GoalID& objective)
  {
    objective_pub.publish(objective);
  }

  void StateMachinePublisher::publishGripperCommand(state_machine::Gripper& gripper_command)
  {
    gripper_pub.publish(gripper_command);
  }

  void StateMachinePublisher::publishTorpedoCommand(state_machine::Torpedo& torpedo_command)
  {
    torpedo_pub.publish(torpedo_command);
  }
}
