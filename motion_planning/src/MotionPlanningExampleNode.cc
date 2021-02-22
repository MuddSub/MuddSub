#include "motion_planning/MotionPlanningPublisher.hh"

using namespace MuddSub::MotionPlanning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_example_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  MotionPlanningPublisher trajectoryPublisher{nh};
  ros::spin();
}
