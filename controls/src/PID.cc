#include "controls/PID.hh"
#include <iostream>
#include <cmath>

namespace MuddSub::Controls
{

double PID::update(double error, double deltaT)
{
  return doUpdate(error, deltaT);
}


double PID::update(double plantState, double setpoint, double deltaT)
{
  double error;

  if(isAngle_)
  {
    auto wrapAngle = [](double num){
      double mod = std::fmod(num, 2*M_PI);
      if(mod < 0)
        return mod + 2*M_PI;
      return mod;
    };

    setpoint = wrapAngle(setpoint);
    plantState = wrapAngle(plantState);

    error = std::fmod(setpoint - plantState, 2*M_PI);
    if(error < 0) error += 2*M_PI;

    double altError = wrapAngle(2*M_PI - error);

    double chosenError = std::min(error, altError);

    // Find the sign by adding to plantstate and seeing if it's right
    double sumError = wrapAngle(plantState + chosenError);
    double diffError = wrapAngle(plantState - chosenError);

    if(sumError < diffError)
      error = chosenError;
    else
      error = -1 * chosenError;
  }
  else
  {
    error = setpoint - plantState;
  }

  return doUpdate(error, deltaT);
}

double PID::doUpdate(double error, double deltaT)
{
  if(deltaT == 0)
    return 0;

  bool gotTuning{true};
  if(tuneFromParams_)
  {
    gotTuning &= nh_.getParam(tuneParamRoot_ + "/kP", kP_);
    gotTuning &= nh_.getParam(tuneParamRoot_ + "/kI", kI_);
    gotTuning &= nh_.getParam(tuneParamRoot_ + "/kD", kD_);
  }
  if(!gotTuning)
  {
    ROS_ERROR("Failed to get tuning from parameter server; make sure it's running");
    ros::Duration(0.5).sleep();
    return 0;
  }

  integralError_ += error * deltaT;


  // First-order IIR filter (infinite impulse response)
  // Essentially acts as a low pass filter with a time constant dependent on
  // the sampling period and the coefficient used (in this case 0.8):
  //    http://www.tsdconseil.fr/tutos/tuto-iir1-en.pdf
  double newDerivative = (error - previousError_) / deltaT;
  derivativeError_ = .8*derivativeError_ + .2*newDerivative;
  previousError_ = error;

  double derivativeTerm = derivativeError_ * kD_;

  auto integralTerm = kI_ * integralError_;
  integralTerm = std::min(integralTerm, windupLimit_);
  integralTerm = std::max(integralTerm, -1*windupLimit_);

  auto result = integralTerm + kD_ * derivativeError_ + kP_*(error);

  return result;
}


void PID::tune(double kP, double kI, double kD)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}
double PID::getKI()
{
    return kI_;
}
double PID::getKP()
{
    return kP_;
}
double PID::getKD()
{
    return kD_;
}
PID::PID(double kP, double kI, double kD, bool isAngle)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = isAngle;
    windupLimit_ = 100;
}

PID::PID(double kP, double kI, double kD, bool isAngle, double windupLimit)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = isAngle;
}

PID::PID(double kP, double kI, double kD)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = false;
}

PID::PID()
{
    kP_ = 0;
    kI_ = 0;
    kD_ = 0;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = false;
}

PID::PID(std::string tuneParamRoot)
{
    tuneParamRoot_ = tuneParamRoot;
    tuneFromParams_ = true;
}
}
