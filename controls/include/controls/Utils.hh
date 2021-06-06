#pragma once

#include "controls/Types.hh"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace MuddSub::Controls
{

nav_msgs::Odometry stateToOdom(const stateVector_t& state);

void broadcastStateAsTF(const stateVector_t& state,
                        const std::string& parentFrame,
                        const std::string& childFrame);



} // namespace MuddSub::Core::Utils

#include "controls/Utils.inl"
