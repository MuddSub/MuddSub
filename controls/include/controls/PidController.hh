#pragma once

#include "ros/ros.h"

/** @brief Simple PID controller

  This PID controller has a first-order filter on the derivative term,
   and windup limits. Since this is only used on our roll and pitch, this
   is sufficient. If to be used on other, less stable axes, filters should
   be reconsidered.
   Notably, if this needs to accomodate changes in setpoint, then the integral
   term needs to be fixed. */
class PidController
{
  /// Gains for proportional, integral, and derivative terms.
  double kP_{0}, kI_{0}, kD_{0};

  /// The 1DOF error (setpoint - plantState)
  double error_;

  /// Integral of error
  double integralError_;

  /// Derivative of error
  double derivativeError_;

  /// The filter is of the form deriv = deriv_current + alpha*deriv_prev. This is that alpha.
  double derivativeFilterAlpha_;

  /// The max (positive OR negative) that an integral can go
  double windupLimit_;

  /// How hard to "push" in the controlled axis
  double controlEffort_;

/// Given the new error and time delay, update integral/derivatives and
/// /returns the computed control effort, for convenience
public:
  inline double update(double err, double deltaT)
  {
    // Discrete derivative
    integralError_ += deltaT * err;

    // Apply windup limits
    if(integralError_ > fabs(windupLimit_))
      integralError_ = windupLimit_;
    else if(integralError_ < -1*fabs(windupLimit_))
      integralError_ = -1*fabs(windupLimit_);

    if(deltaT == 0)
    {
      ROS_WARN("PID DeltaT Was Zero");
    }
    else
    {
      // 1st order filter on derivative
      derivativeError_ = (err - error_)/deltaT  +
                          derivativeFilterAlpha_ * derivativeError_;
    }

    error_ = err;
    // P + I + D == PID?
    controlEffort_ = kP_ * error_ + kI_ * integralError_ + kD_ * derivativeError_;
  }
};
