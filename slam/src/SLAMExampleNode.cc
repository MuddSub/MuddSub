#include "slam/SLAMPublisher.hh"

using namespace MuddSub::SLAM;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  SLAMPublisher slamPublisher{nh};
  ros::spin();
}
