#pragma once

#include "controls/DecoupledLQR.hh"
#include "controls/State.h"
#include <nav_msgs/Odometry.h>
#include "controls/Controller.hh"
#include "controls/Types.hh"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/ros.h>


namespace MuddSub::Controls
{

class ControlDispatch
{
public:
  ControlDispatch();

private:
  std::shared_ptr<Controller> controller_;
  VehicleDynamics dynamics_;

  ros::NodeHandle nh_;

  ros::Subscriber setpointSub_;
  ros::Subscriber plantStateSub_;

  stateVector_t plantState_;

  void plantCallback(const nav_msgs::Odometry& msg);
  void setpointCallback(const controls::State& state);

  stateVector_t odomToState(const nav_msgs::Odometry& msg);

};

}
