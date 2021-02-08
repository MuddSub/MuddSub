#include "motion_planning/TrajectoryPublisher.hh"

namespace MuddSub::MotionPlanning
{
  TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle n):
    n_(n)
  {
    trajectory_pub = n_.advertise<nav_msgs::Path>("motion_planning/trajectory", 1000);
  }

  void TrajectoryPublisher::publishTrajectory(nav_msgs::Path& trajectory)
  {
    trajectory_pub.publish(trajectory);
  }
}
