#pragma once

#include "controls/LQRController.hh"
#include "controls/VehicleDynamics.hh"
#include "controls/PidController.hh"

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <map>
#include <algorithm>
#include <cmath>

//**

/** @brief The controls package contains code related to vehicle dynamics and motion controls.

The primary controller implemented is 8DOF LQR with 2DOF PID.
We also implement fossen's equations of motion for both simulation and optimal controls.
*/
namespace MuddSub::Controls
{

/// 12DOF coupled nonlinear vehicle dynamics
class VehicleDynamics;

/** @brief a 12DOF optimal controller, using LQR and PID.
This decoupled controller implements 8DOF LQR control, handling position and velocity
in surge, sway, heave, and yaw. This is what is needed for path following and primary
navigation.

Roll and pitch is handled with simple PID controllers, which are generally inactive
as the robot is quite stable in those axes (+1 for good mechanical design making software easier).
*/
class DecoupledLQR : public ct::core::Controller<12, 6>
{

public:
  DecoupledLQR() = default;
  DecoupledLQR(const DecoupledLQR&) = default;

  /// Dimension of AUV state: [x,y,z,roll,pitch,yaw,x',y',z',roll',pitch',yaw']
  constexpr static unsigned int stateDim = 12;

  /// Dimension of control: 6DOF wrench
  constexpr static unsigned int controlDim = 6;

  /// LQR has state [x,y,z,yaw, x', y', z', yaw']
  static constexpr unsigned int stateDimLQR = 8;

  /// LQR control is 4DOF wrench: [x,y,z,yaw]
  static constexpr unsigned int controlDimLQR = 4;

  /// The state vector is by default a 12D vector, representing the position
  /// and attitude in 6DOF, and all of those velocities.
  using stateVector_t = ct::core::StateVector<stateDim, double>;

  /// The control vector is a 6-vector, which is just the wrench (forces/torques)
  /// in 6DOF
  using controlVector_t = ct::core::ControlVector<controlDim, double>;

  /// The control toolbox has their own time type, which we use for properness.
  using ctTime_t = ct::core::ControlledSystem<stateDim, controlDim, double>::time_t;

  /// Clone the object
  DecoupledLQR* clone() const;

  /// Given the state and stored setpoint, compute the error.
  stateVector_t computeError(const stateVector_t& state) const;

  /// Given the current error, find the control action (6DOF wrench.)
  /// This implements the described controller.
  void computeControl(const stateVector_t& state,
                      const double&,
                      controlVector_t& controlAction);

  /// Set the 12DOF vehicle setpoint
  /// @param setpoint: Desired location of robot
  inline void setSetpoint(const stateVector_t& setpoint)
  {
    ROS_INFO("Setpoint updated");
    setpoint_ = setpoint;
  };

  /// The controller needs knowledge of the vehicle dynaics to solve the
  /// optimal control problem
  /// @param dynamicsPtr: shared_ptr to instance of vehicle dynamics. Only one such instance should be present at a time.
  inline void setDynamics(std::shared_ptr<VehicleDynamics> dynamicsPtr)
  {
    vehicleDynamics_ = dynamicsPtr;
  }


private:

  /// Simple PID controllers for roll and pitch
  PidController rollPid_, pitchPid_;

  /// 8DOF controller for x,y,z,yaw and those velocities
  LQRController<stateDimLQR, controlDimLQR> lqr8DoF_;

  /// Stores the 12DOF dynamics of the robot.
  std::shared_ptr<VehicleDynamics> vehicleDynamics_;

  /// Current 12DOF setpoint (where we want to be).
  stateVector_t setpoint_;

  /// Get the 8DOF A matrix from the 12DOF A matrix (throw out roll/pitch)
  /// @param A: 12DOF linearized A matrix
  /// @returns partitionAMatrix: the partitioned 8x8 A matrix
  Eigen::Matrix<double, stateDimLQR, stateDimLQR> partitionAMatrix(const Eigen::Matrix<double, stateDim, stateDim>& A);

  /// Get the 8DOF B matrix from the 12DOF B matrix (throw out roll/pitch).
  /// @param B: the 12x6 lienarized B matrix
  /// @returns partitionBMatrix: the 8x6 (8 DOF state, 6 control inputs)
  Eigen::Matrix<double, stateDimLQR, controlDimLQR> partitionBMatrix(const Eigen::Matrix<double, stateDim, controlDim>& B);

  /// For integrating and derivatives: Store the previous time
  double previousTime_;

};

} // namespace MuddSub::Controls
