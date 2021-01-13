#pragma once

#include <ct/core/core.h>

namespace MuddSub::Controls
{

/// Dimension of AUV state: [x,y,z,roll,pitch,yaw,x',y',z',roll',pitch',yaw']
constexpr unsigned int stateDim = 12;

/// Dimension of control: 6DOF wrench
constexpr unsigned int controlDim = 6;

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


}
