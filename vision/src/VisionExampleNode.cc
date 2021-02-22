#include "vision/VisionPublisher.hh"

using namespace MuddSub::Vision;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  VisionPublisher visionPublisher{nh, "test_camera"};
  ros::spin();
}
