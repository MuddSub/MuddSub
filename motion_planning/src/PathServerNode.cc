#include "motion_planning/PathServer.hh"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PathServer");
  ros::NodeHandle nh;
  MuddSub::MotionPlanning::PathServer actionServer("PathServer");
  ros::spin();
  return 0;
}
