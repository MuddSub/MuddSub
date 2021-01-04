#pragma once

#include <ros/ros.h>
#include "controls/State.h"
#include "controls/DecoupledLQR.hh"
#include <memory>
#include "controls/StateToOdomPublisher.hh"
#include "controls/Types.hh"

namespace MuddSub::Controls
{

/** @brief Simulates vehicle dynamics and control systems.

SimulationDynamics implements the decoupled LQR controller
to control a nonlinear system described by Fossen's equations of motion
for an underwater vehicle. At each time step, the control action is computed
and applied to the system. Then, the results are propogated forward to compute
the new state.
*/
class SimulationDynamics
{

public:
  SimulationDynamics();

  /// Subscribes to a ROS message setting the setpoint (as a 12-vector)
  /// @param state: the desired state of the robot
  void setpointCB(const controls::State& state);

  /// Propogates the dynamics and computes the next state derivative
  void runOnce();

  /// Updates per second
  double rate_{10};

private:

  /// Description of nonlinear system
  VehicleDynamics dynamics_;

  /// Linear controller
  std::shared_ptr<DecoupledLQR> controller_;

  /// Subscribe to the setpoint
  ros::Subscriber setpointSub_;
  ros::NodeHandle nh_;

  /// Publisher for the current state
  StateToOdomPublisher statePub_;

  /// Used to caluclate the delta in time for propogating state derivative
  double prevTime_{0};

  stateVector_t state_{stateVector_t::Zero()};
  stateVector_t stateDerivative_{stateVector_t::Zero()};

};

}
