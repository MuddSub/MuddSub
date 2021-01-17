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
#include <geometry_msgs/Wrench.h>


namespace MuddSub::Controls
{

class ControlDispatch
{
public:
  ControlDispatch();

  void publishControl(const controlVector_t& control);

  void iterate();

private:
  std::shared_ptr<Controller> controller_;
  VehicleDynamics dynamics_;

  ros::NodeHandle nh_;

  ros::Subscriber setpointSub_;
  ros::Subscriber plantStateSub_;

  ros::Publisher controlPub_;

  stateVector_t plantState_{stateVector_t::Zero()};

  void plantCallback(const nav_msgs::Odometry& msg);
  void setpointCallback(const controls::State& state);

  stateVector_t odomToState(const nav_msgs::Odometry& msg);
  Eigen::IOFormat eigenInLine{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", ";"};

};

}
