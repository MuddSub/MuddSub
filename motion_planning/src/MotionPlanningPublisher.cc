#include "motion_planning/MotionPlanningPublisher.hh"

namespace MuddSub::MotionPlanning
{
  MotionPlanningPublisher::MotionPlanningPublisher(ros::NodeHandle n):
    n_(n)
  {
    motion_planning_pub = n_.advertise<nav_msgs::Path>("motion_planning/trajectory", 1000);
  }

  void MotionPlanningPublisher::publishTrajectory(nav_msgs::Path& trajectory)
  {
    motion_planning_pub.publish(trajectory);
  }
}
