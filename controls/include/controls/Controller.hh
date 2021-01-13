#pragma once

#include "controls/PidController.hh"
#include "controls/DecoupledLQR.hh"
#include "controls/Types.hh"
#include "controls/VehicleDynamics.hh"


namespace MuddSub::Controls
{

class VehicleDynamics;

class Controller : public ct::core::Controller<stateDim, controlDim>
{
public:

  /// Set the 12DOF vehicle setpoint
  /// @param setpoint: Desired location of robot
  inline void setSetpoint(const stateVector_t& setpoint)
  {
    setpoint_ = setpoint;
  };


  /// Returns the 12DOF setpoint.
  inline stateVector_t getSetpoint()
  {
    return setpoint_;
  };

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

protected:

  /// Current 12DOF setpoint.
  stateVector_t setpoint_;

  // Is it model based? If so, you must set the dynamics.
  bool modelBased_{false};

  /// Stores the dynamics of the robot.
  std::shared_ptr<VehicleDynamics> vehicleDynamics_;

};
}
