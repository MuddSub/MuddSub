#include "motion_planning/PathServer.hh"


namespace MuddSub::MotionPlanning
{

PathServer::PathServer(std::string name):
  actionServer_(nh_, name, boost::bind(&PathServer::createPath, this, _1), false)
{
  actionServer_.start();
}

void PathServer::createPath(const motion_planning::CreatePathGoalConstPtr& goal)
{
  result_.path_waypoints = goal->target_waypoints;
  result_.node_count = goal->target_waypoints.size();
  actionServer_.setSucceeded(result_);
}

} // namespace MuddSub::MotionPlanning
