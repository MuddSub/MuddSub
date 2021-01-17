#include "slam/SLAMPublisher.hh"

using namespace MuddSub::SLAM;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SLAMPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  SLAMPublisher slamPublisher{nh};
  ros::spin();
}
