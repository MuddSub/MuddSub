#include "slam/SLAMPublisher.hh"

namespace MuddSub::SLAM
{
  SLAMPublisher::SLAMPublisher(ros::NodeHandle n):
    n_(n)
  {
    statePub_ = n_.advertise<nav_msgs::Odometry>("slam/robot/state", 1000);
    mapPub_ = n_.advertise<slam::Map>("slam/map", 1000);
  }

  void SLAMPublisher::publishState(nav_msgs::Odometry& odometry)
  {
    statePub_.publish(odometry);
  }

  void SLAMPublisher::publishMap(slam::Map& map)
  {
    mapPub_.publish(map);
  }
}
