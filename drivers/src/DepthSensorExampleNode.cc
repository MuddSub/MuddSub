#include "drivers/DepthSensorPublisher.hh"

using namespace MuddSub::DepthSensor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_sensor_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  DepthSensorPublisher depthSensorPublisher{nh};
  ros::spin();
}
