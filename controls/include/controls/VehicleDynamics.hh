#pragma once

#include <ct/core/core.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <map>
#include <algorithm>
#include "controls/DecoupledLQR.hh"
#include "geometry_msgs/Pose.h"
#include <memory>

namespace MuddSub::Controls
{

/** @brief Implements Fossen's equations of motion for 12DOF underwater vehicle.

This class implements the 12DOF robot dynamics primarily described by
Fossen, 2011: Handbook of Marine Craft Hydrodynamics and Motion Control,
and made clearer by Chin, 2013: Computer Aided Control Systems Design (beware, the Chin book has errors).

These dynamics are used to compute the motion of the AUV in simulation (as the simulator does not natively
support vehicle dynamics) and for computing control systems.

Built by the muddsub_primary and muddsub_sim targets.
*/
class VehicleDynamics : public ct::core::ControlledSystem<12, 6, double>
{

public:

  /// Dimension of AUV state: [x,y,z,roll,pitch,yaw,x',y',z',roll',pitch',yaw']
  constexpr static unsigned int stateDim = 12;

  /// Dimension of control: 6DOF wrench
  constexpr static unsigned int controlDim = 6;

  /// The state vector is by default a 12D vector, representing the position
  // and attitude in 6DOF, and all of those velocities.
  using stateVector_t = ct::core::StateVector<stateDim, double>;

  /// The control vector is a 6-vector, which is just the wrench (forces/torques)
  // in 6DOF
  using controlVector_t = ct::core::ControlVector<controlDim, double>;

  /// The control toolbox has their own time type, which we use for properness.
  using ctTime_t = ct::core::ControlledSystem<stateDim, controlDim, double>::time_t;

  /// Type of the controller.
  using controllerPtr_t = std::shared_ptr<ct::core::Controller<stateDim, controlDim, double>>;

  /// The constructor will read all parameters from a ROS parameter server.
  VehicleDynamics();

  VehicleDynamics(const VehicleDynamics& other) = default;

  // Raw pointer clone is required by control toolbox, but discouraged
  VehicleDynamics* clone() const;

  // Use this instead!
  std::shared_ptr<VehicleDynamics> smartClone() const;

  /// Given the current state and specified control vector, compute the deriviative of the state.
  // This effectively implements the x'= Ax + Bu state-space equation.
  /// @param state: Current state of robot
  /// @param control: The applied control effort
  /// @returns derivative: the CT state derivative
  virtual void computeControlledDynamics(const stateVector_t &state,
                                         const ctTime_t&,
                                         const controlVector_t &control,
                                         stateVector_t &derivative);

  /// Given the state and time, compute the control, apply that control,
  // then compute the dynamics.
  /// @param state: the current state of robot
  /// @param t: Current time
  /// @param deriv: the calculated CT state derivative
  void computeDynamics(const stateVector_t& state, const ctTime_t& t, stateVector_t& deriv);

  /// Set the controller, and mark it as set.
  /// @param controller: a pointer to the controller object. Ideally there should only be 1 instance at a time.
  void setController(const controllerPtr_t& controller);

private:

  /// Robot dynamics properties, where all values are fetched from a
  /// ROS parameter sever.
  typedef struct RobotInfo_st
  {
    std::string name_;
    Eigen::Matrix3d inertiaTensor_;
    std::map<std::string, double> addedMass_, linearDampingCoefficients_,
                                  centerOfGravity_, centerOfBuoyancy_, inertia_;

    Eigen::Vector3d centerOfGravityVector_, centerOfBuoyancyVector_;
    double mass_, buoyancy_;
  } RobotInfo;

  RobotInfo robotInfo_;


  constexpr static double gravity_{9.8};

  /// Excecution rate. Loaded from ROS parameter server. Ballpark ~20Hz.
  double rate_;

  /// State 12-vector (6DOF position, 6DOF velocity)
  stateVector_t x_;

  /// Control input 6-vector (wrench in 6D)
  controlVector_t u_;

  /// ROS NodeHandle
  ros::NodeHandle nh_;

  // An identity matrix, because it's useful.
  Eigen::Matrix3d I_{Eigen::Matrix3d::Identity()};

  /// Given the velocities, compute the coriolis and centripital forces
  /// @param linearVelocity: The 3DOF linear velocity of the robot
  /// @param angularVelocity: The 3DOF angular velocity of the robot
  Eigen::Matrix<double, 6, 6> buildCMatrix(const Eigen::Vector3d& linearVelocity,
                                           const Eigen::Vector3d& angularVelocity);

  /// Find the general mass, including real mass and hydrodynamic added mass
  Eigen::Matrix<double, 6, 6> buildMassMatrix();

  /// Find the transformation between the robot frame and world frame.
  /// @param attitude: The 3DOF attitude of the robot
  Eigen::Matrix<double, 6, 6> buildJnMatrix(const Eigen::Vector3d& attitude);

  /// Find the effect of gravity and buoyancy on the robot's position and attitude
  /// @param attitude: The 3DOF attitude of the robot
  Eigen::Matrix<double, 6, 1> buildGravityMatrix(const Eigen::Vector3d& attitude);

  /// Find the skew-symmetric matrix for computing quick cross products with 3-vectors.
  Eigen::Matrix3d S(const Eigen::Vector3d& vec) const;

  /// Whether or not the controller has been set. Avoids nonsense calls to the controller
  bool controllerSet_{0};

};

}
