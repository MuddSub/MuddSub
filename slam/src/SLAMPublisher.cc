#include "slam/SLAMPublisher.hh"

namespace MuddSub::SLAM
{
  SLAMPublisher::SLAMPublisher(ros::NodeHandle n):
    n_(n)
  {
    state_pub = n_.advertise<nav_msgs::Odometry>("slam/robot/state", 1000);
    map_pub = n_.advertise<slam::Map>("slam/map", 1000);
  }

  void SLAMPublisher::publishState(nav_msgs::Odometry& odometry)
  {
    state_pub.publish(odometry);
  }

  void SLAMPublisher::publishMap(slam::Map& map)
  {
    map_pub.publish(map);
  }
}
