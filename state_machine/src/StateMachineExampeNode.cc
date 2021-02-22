#include "state_machine/StateMachinePublisher.hh"

using namespace MuddSub::StateMachine;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_machine_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  StateMachinePublisher stateMachinePublisher{nh};
  ros::spin();
}
