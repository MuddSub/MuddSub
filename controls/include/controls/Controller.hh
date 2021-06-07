#pragma once

#include "controls/Types.hh"
#include "controls/VehicleDynamics.hh"
#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>

namespace MuddSub::Controls
{

class VehicleDynamics;

class Controller : public ct::core::Controller<stateDim, controlDim>
{
public:

  Controller();

  /// Set the 12DOF vehicle setpoint
  /// @param setpoint: Desired location of robot
  void setSetpoint(const stateVector_t& setpoint)
  {
    setpoint_ = setpoint;
  };

  void setPlantState(const stateVector_t& plantState)
  {
    plantState_ = plantState;
  };

  /// Returns the 12DOF setpoint.
  inline stateVector_t getSetpoint()
  {
    return setpoint_;
  };

  stateVector_t getError();

  /// Get the error (setpoint - plantstate) with angles corrected
  stateVector_t getError(const stateVector_t& plantState);

  /// The controller needs knowledge of the vehicle dynamics to solve the
  /// control problem, in the case of a model-based controller.
  /// @param dynamicsPtr: shared_ptr to instance of vehicle dynamics. Only one such instance should be present at a time.
  inline void setDynamics(std::shared_ptr<VehicleDynamics> dynamicsPtr)
  {
    vehicleDynamics_ = dynamicsPtr;
  }

  std::shared_ptr<VehicleDynamics> getVehicleDynamics()
  {
    return vehicleDynamics_;
  }

  // Note: Don't use this unless the class you're deriving from has implemented
  // a real version. This is a compile hack.
  Controller* clone() const
  {
    return const_cast<Controller*>(this);
  };

  void reset(){};

protected:

  /// Current 12DOF setpoint.
  stateVector_t setpoint_;

  stateVector_t plantState_;
  
  // Is it model based? If so, you must set the dynamics.
  bool modelBased_{false};

  /// Stores the dynamics of the robot.
  std::shared_ptr<VehicleDynamics> vehicleDynamics_;

  /// For computing dT: Store the previous time
  double previousTime_;

  double angleError(double setpoint, double plant);

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf2Listener_;

};
}
