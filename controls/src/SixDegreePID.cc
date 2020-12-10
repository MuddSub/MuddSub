//
// Created by tanvi on 11/21/2020.
//
#include "PID.h"
#include "SixDegreePID.h"
#include <vector>

SixDegreePID::SixDegreePID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD)
{
    roll_ = PID(kP.at(0), kI.at(0), kD.at(0), true);
    pitch_ = PID(kP.at(1), kI.at(1), kD.at(1), true);
    yaw_ = PID(kP.at(2), kI.at(2), kD.at(2), true);

    surge_ = PID(kP.at(3), kI.at(3), kD.at(3));
    sway_ = PID(kP.at(4), kI.at(4), kD.at(4));
    heave_ = PID(kP.at(5), kI.at(5), kD.at(5));

}

PID& SixDegreePID::getRoll()
{
    return roll_;
}

PID& SixDegreePID::getPitch()
{
    return pitch_;
}

PID& SixDegreePID::getYaw()
{
    return yaw_;
}

PID& SixDegreePID::getSurge()
{
    return surge_;
}

PID& SixDegreePID::getSway()
{
    return sway_;
}

PID& SixDegreePID::getHeave()
{
    return heave_;
}


std::vector<double> SixDegreePID::update(std::vector<double> plantState, std::vector<double> setPoint,  double dT)
{
    std::vector<double> effort;
    effort.push_back(roll_.update(plantState.at(0),setPoint.at(0),dT));
    effort.push_back(pitch_.update(plantState.at(1),setPoint.at(1),dT));
    effort.push_back(yaw_.update(plantState.at(2),setPoint.at(2),dT));

    effort.push_back(surge_.update(plantState.at(3),setPoint.at(3),dT));
    effort.push_back(sway_.update(plantState.at(4),setPoint.at(4),dT));
    effort.push_back(heave_.update(plantState.at(5),setPoint.at(5),dT));

    return effort;

}

void SixDegreePID::tuneController(double kP, double kI, double kD,  PID& controller)
{
    controller.tune(kP, kI, kD);

}
