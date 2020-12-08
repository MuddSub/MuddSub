//
// Created by tanvi on 11/21/2020.
//
#include "PID.h"
#include "six_deg_PID.h"
#include <vector>

six_deg_PID::six_deg_PID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD)
{
    roll_ = PID(kP.at(0), kI.at(0), kD.at(0), true);
    pitch_ = PID(kP.at(1), kI.at(1), kD.at(1), true);
    yaw_ = PID(kP.at(2), kI.at(2), kD.at(2), true);

    x_ = PID(kP.at(3), kI.at(3), kD.at(3));
    y_ = PID(kP.at(4), kI.at(4), kD.at(4));
    z_ = PID(kP.at(5), kI.at(5), kD.at(5));

}

PID six_deg_PID::getRoll()
{
    return roll_;
}

PID six_deg_PID::getPitch()
{
    return pitch_;
}

PID six_deg_PID::getYaw()
{
    return yaw_;
}

PID six_deg_PID::getX()
{
    return x_;
}

PID six_deg_PID::getY()
{
    return y_;
}

PID six_deg_PID::getZ()
{
    return z_;
}

void six_deg_PID::tuneAngle(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD)
{
    roll_.tune(kP.at(0), kI.at(0), kD.at(0));
    pitch_.tune(kP.at(1), kI.at(1), kD.at(1));
    yaw_.tune(kP.at(2), kI.at(2), kD.at(2));
}

void six_deg_PID::tunePosition(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD)
{
    x_.tune(kP.at(0), kI.at(0), kD.at(0));
    y_.tune(kP.at(1), kI.at(1), kD.at(1));
    z_.tune(kP.at(2), kI.at(2), kD.at(2));
}

std::vector<double> six_deg_PID::update(std::vector<double> plantState, std::vector<double> setPoint,  double dT)
{
    std::vector<double> effort;
    effort.push_back(roll_.update(plantState.at(0),setPoint.at(0),dT));
    effort.push_back(pitch_.update(plantState.at(1),setPoint.at(1),dT));
    effort.push_back(yaw_.update(plantState.at(2),setPoint.at(2),dT));

    effort.push_back(x_.update(plantState.at(3),setPoint.at(3),dT));
    effort.push_back(y_.update(plantState.at(4),setPoint.at(4),dT));
    effort.push_back(z_.update(plantState.at(5),setPoint.at(5),dT));

    return effort;

}

