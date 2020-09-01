#pragma once

#include <nav_msgs/Odometry.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace MuddSub::Core
{
class OdomTfBroadcaster
{

public:
  OdomTfBroadcaster(ros::NodeHandle nh, const std::string& odomTopic, const std::string parentFrame, const std::string& childFrame);
  OdomTfBroadcaster() = delete;
  OdomTfBroadcaster(const OdomTfBroadcaster&) = delete;

private:
  void odomCallback(const nav_msgs::Odometry& msg);
  ros::Subscriber odomSub_;
  std::string topic_, parentFrame_, childFrame_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;
};
} // namespace MuddSub::Core
