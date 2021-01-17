#include "depth_sensor/DepthSensorPublisher.hh"

using namespace MuddSub::DepthSensor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DepthSensorPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  DepthSensorPublisher depthSensorPublisher{nh};
  ros::spin();
}
