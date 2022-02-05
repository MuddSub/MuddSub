#pragma once

#include "controls/DecoupledLQR.hh"
#include "controls/SixDegreePID.hh"
#include "controls/State.h"
#include "controls/Controller.hh"
#include "controls/Types.hh"
#include "controls/Utils.hh"
#include "controls/ControlsPublisher.hh"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
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

  void setZeroToCurrent();
  void setZero(const nav_msgs::Odometry& odom);

private:
  std::shared_ptr<Controller> controller_;
  VehicleDynamics dynamics_;

  ros::NodeHandle nh_;

  ros::Subscriber setpointSub_;
  ros::Subscriber plantStateSub_;
  ros::Subscriber resetSub_;
  ros::Subscriber zeroHereSub_;
  ros::Subscriber zeroSub_;

  // ros::Publisher controlPub_;

  stateVector_t plantState_{stateVector_t::Zero()};
  stateVector_t plantZero_{stateVector_t::Zero()};
  stateVector_t setpoint_{stateVector_t::Zero()};

  void plantCallback(const nav_msgs::Odometry& msg);
  void resetCallback(const std_msgs::Empty& msg);
  void setpointCallback(const controls::State& state);

  void zeroCurrentCallback(const std_msgs::Empty&)
  {
    setZeroToCurrent();
  };

  void zeroToCallback(const nav_msgs::Odometry& msg)
  {
    setZero(msg);
  }

  stateVector_t odomToState(const nav_msgs::Odometry& msg);
  Eigen::IOFormat eigenInLine{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", ";"};

  ControlsPublisher controlsPublisher_;
};

}
