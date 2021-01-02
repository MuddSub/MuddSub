//
// Created by tanvi on 11/21/2020.
//

#pragma once
#include "controls/PID.hh"
#include <vector>

class SixDegreePID
{
private:
    PID roll_ = PID();
    PID pitch_ = PID();
    PID yaw_ = PID();

    PID surge_ = PID();
    PID sway_ = PID();
    PID heave_ = PID();

public:
    SixDegreePID();
    SixDegreePID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD);

    PID& getRoll();
    PID& getPitch();
    PID& getYaw();

    PID& getSurge();
    PID& getSway();
    PID& getHeave();


    void tuneController(double kP, double kI, double kD,  PID& controller);

    //format of setPoint and plantState is <surge, sway, heave, roll, pitch, yaw >
    void update(VehicleDynamics::stateVector_t setPoint, VehicleDynamics::stateVector_t plantState, double dT, VehicleDynamics::controlVector_t control);
};
