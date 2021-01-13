#include "controls/PID.hh"
#include <iostream>

namespace MuddSub::Controls
{
double PID::update(double plantState, double setPoint, double deltaT)
{
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
    }

    double error = setPoint - plantState;
    if(isAngle_)
    {
        double explementaryError  = (360 - error)*-1;
        if(std::abs(explementaryError) < std::abs(error))
        {
            error = explementaryError;
        }
    }

    integralError_ += error * deltaT;
    integralError_ = std::min(integralError_, windupLimit_);
    integralError_ =  std::max(integralError_, -1*windupLimit_);

    // First-order IIR filter (infinite impulse response)
    derivativeError_ = .7*derivativeError_ + .3*(error - previousError_) / deltaT;
    previousError_ = error;

    auto result = kI_ * integralError_ + kD_ * derivativeError_ + kP_*(error);


    if(tuneParamRoot_ == "/heave")
    {
      ROS_INFO("Prop: %f, Int: %f, Deriv: %f, dt %f", kP_*error, kI_ * integralError_, kD_ * derivativeError_, deltaT);
    }
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
