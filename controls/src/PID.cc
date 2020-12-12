//
// Created by tanvi on 10/30/2020.
//

#include "controls/PID.hh"
#include <iostream>

double PID::update(double plantState, double setPoint, double deltaT)
{
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

    double derivativeError = (error - previousError_) / deltaT;
    previousError_ = error;
    return kI_ * integralError_ + kD_ * derivativeError + kP_*(error);
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
    windupLimit_ = windupLimit;
}

PID::PID(double kP, double kI, double kD)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = false;
    windupLimit_ = 100;
}

PID::PID()
{
    kP_ = 0;
    kI_ = 0;
    kD_ = 0;
    previousError_ = 0;
    integralError_ = 0;
    isAngle_ = false;
    windupLimit_ = 100;
}
