#pragma once

#include <motion_planning/CreatePathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>


namespace MuddSub::MotionPlanning
{

using Server = actionlib::SimpleActionServer<motion_planning::CreatePathAction>;

class PathServer
{
public:
  PathServer() = delete;
  PathServer(std::string name);

protected:
  ros::NodeHandle nh_;
  void createPath(const motion_planning::CreatePathGoalConstPtr& goal);

  // Actionlib messages
  motion_planning::CreatePathFeedback feedback_;
  motion_planning::CreatePathResult result_;

  Server actionServer_;

};
} // namespace MuddSub::MotionPlanning
