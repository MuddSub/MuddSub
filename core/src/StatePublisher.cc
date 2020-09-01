  #include "core/StatePublisher.hh"

namespace MuddSub::Core
{

StatePublisher::StatePublisher()
{
}

void StatePublisher::addBroadcaster(const std::string& odomTopic, const std::string parentFrame, const std::string& childFrame)
{
    tfBroadcasters_.push_back(std::make_unique<OdomTfBroadcaster>(nh_, odomTopic, parentFrame, childFrame));
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle nh;
  std::string description;
  while(ros::ok())
  {
    nh.getParam("/robot_description", description);
    ROS_INFO("Descritpions: %s", description);
  }
  MuddSub::Core::StatePublisher statePub;
  statePub.addBroadcaster("topic", "parent", "child");
  ros::Rate loopRate(100);
  ros::spin();
}
