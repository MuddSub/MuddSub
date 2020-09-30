#pragma once

/** @brief Simple PID controller

  This PID controller has a first-order filter on the derivative term,
   and windup limits. Since this is only used on our roll and pitch, this
   is sufficient. If to be used on other, less stable axes, filters should
   be reconsidered.
*/
class PidController
{
private:
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
  inline void tune(double kP, double kI, double kD) noexcept
  {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
  };

  inline void setKP(const double kP) noexcept
  {
    kP_ = kP;
  };

  inline void setKI(const double kI) noexcept
  {
    kI_ = kI;
  };

  inline void setKD(const double kD) noexcept
  {
    kD_ = kD;
  };

  inline double getKP() const noexcept
  {
    return kP_;
  }

  inline double getKI() const noexcept
  {
    return kI_;
  }

  inline double getKD() const noexcept
  {
    return kI_;
  }

  /** @brief Updates errors; computes and returns the effort from the controller.

      \exception std::out_of_range if windupLimit_ is negative.
  */
  double update(const double err, const double deltaT)
  {
    if(windupLimit_ < 0)
      throw std::out_of_range("Windup limit must be non-negative");

    // Discrete derivative
    integralError_ += deltaT * err;

    // Apply windup limits
    if(integralError_ > windupLimit_)
      integralError_ = windupLimit_;
    else if(integralError_ < -1*windupLimit_)
      integralError_ = -1*windupLimit_;

    // 1st order filter on derivative
    derivativeError_ = (err - error_)/deltaT  +
                        derivativeFilterAlpha_ * derivativeError_;
    error_ = err;
    // P + I + D == PID?
    controlEffort_ = kP_ * error_ + kI_ * integralError_ + kD_ * derivativeError_;
  }
};
