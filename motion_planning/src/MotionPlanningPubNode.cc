#include "motion_planning/MotionPlanningPublisher.hh"

using namespace MuddSub::MotionPlanning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MotionPlanningPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  MotionPlanningPublisher trajectoryPublisher{nh};
  ros::spin();
}
