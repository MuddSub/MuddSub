#include "vision/VisionPublisher.hh"

using namespace MuddSub::Vision;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VisionPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  VisionPublisher visionPublisher{nh, "test_camera"};
  ros::spin();
}
