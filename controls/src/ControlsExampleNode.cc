#include "controls/ControlsPublisher.hh"

using namespace MuddSub::Controls;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controls_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ControlsPublisher controlsPublisher{nh};
  ros::spin();
}
