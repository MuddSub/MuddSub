#include "motion_planning/TrajectoryPublisher.hh"

using namespace MuddSub::MotionPlanning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TrajectoryPubNode", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  TrajectoryPublisher trajectoryPublisher{nh};
  ros::spin();
}
