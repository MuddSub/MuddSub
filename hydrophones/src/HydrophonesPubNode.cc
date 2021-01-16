#include "hydrophones/HydrophonesPublisher.hh"

using namespace MuddSub::Hydrophones;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HydrophonesPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  HydrophonesPublisher hydrophonesPublisher{nh};
  ros::spin();
}
