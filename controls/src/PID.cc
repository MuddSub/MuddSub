//
// Created by tanvi on 10/30/2020.
//

#include "PID.h"
#include <iostream>

double PID::update(double plantState, double setPoint, double deltaT)
{
    double error = setPoint - plantState;
    if(angle_)
    {

        double otherError = (360 - error)*-1;
        if(std::abs(otherError) < std::abs(error))
        {
            error = otherError;
        }

        //std::cout<<"error"<<error<<' '<<goal<<' '<<current<<std::endl;
    }
    integralError_ += error * deltaT;
    double derivativeError = (error + previousError_) / 2;
    previousError_ = error;
    return kI_ * integralError_ + kD_ * derivativeError + kP_*(error);
}


void PID::tune(double kP, double kI, double kD)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}
double PID::getkI()
{
    return kI_;
}
double PID::getkP()
{
    return kP_;
}
double PID::getkD()
{
    return kD_;
}
PID::PID(double kP, double kI, double kD, bool angle)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    angle_ = angle;
}

PID::PID(double kP, double kI, double kD)
{
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    previousError_ = 0;
    integralError_ = 0;
    angle_ = false;
}

PID::PID()
{
    kP_ = 0;
    kI_ = 0;
    kD_ = 0;
    previousError_ = 0;
    integralError_ = 0;
    angle_ = false;
}
