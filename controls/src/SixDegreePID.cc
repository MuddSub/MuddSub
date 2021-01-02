//
// Created by tanvi on 11/21/2020.
//
#include "controls/PID.hh"
#include "controls/SixDegreePID.hh"
#include <vector>

SixDegreePID::SixDegreePID():
  roll_("/roll"), pitch_("/pitch"), yaw_("/yaw"),
  surge_("/surge"), sway_("/sway"), heave_("/heave")
{
}

SixDegreePID::SixDegreePID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD)
{
    surge_ = PID(kP[0], kI[0], kD[0]);
    sway_ = PID(kP[1], kI[1], kD[1]);
    heave_ = PID(kP[2], kI[2], kD[2]);

    roll_ = PID(kP[3], kI[3], kD[3], true);
    pitch_ = PID(kP[4], kI[4], kD[4], true);
    yaw_ = PID(kP[5], kI[5], kD[5], true);


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


void SixDegreePID::update(stateVector_t& plantState, stateVector_t& setPoint,  double dT, VehicleDynamics::controlVector_t& control)
{

    control[0] = surge_.update(plantState[0],setPoint[0],dT);
    control[1] = sway_.update(plantState[1],setPoint[1],dT);
    control[2] = heave_.update(plantState[2],setPoint[2],dT);

    control[3] = roll_.update(plantState[3],setPoint[3],dT);
    control[4] = pitch_.update(plantState[4],setPoint[4],dT);
    control[5] = yaw_.update(plantState[5],setPoint[5],dT);


}

void SixDegreePID::tuneController(double kP, double kI, double kD,  PID& controller)
{
    controller.tune(kP, kI, kD);

}
