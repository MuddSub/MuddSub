#pragma once

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "core/OdomTfBroadcaster.hh"
#include <vector>
#include <memory>
#include <tf/transform_broadcaster.h>

namespace MuddSub::Core
{
class StatePublisher{

public:

  StatePublisher();

  void addBroadcaster(const std::string& odomTopic, const std::string parentFrame, const std::string& childFrame);

private:

  ros::NodeHandle nh_;
  tf::TransformBroadcaster tfBroadcaster;

  std::vector<std::unique_ptr<OdomTfBroadcaster>> tfBroadcasters_;
};


} // namespace MuddSub::Core
