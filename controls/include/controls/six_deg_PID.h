//
// Created by tanvi on 11/21/2020.
//

#ifndef CONTROLS_SIX_DEG_PID_H
#define CONTROLS_SIX_DEG_PID_H
#include "PID.h"
#include <vector>

class six_deg_PID
{
private:
    PID roll_ = PID();
    PID pitch_ = PID();
    PID yaw_ = PID();

    PID x_ = PID();
    PID y_ = PID();
    PID z_ = PID();

public:

    six_deg_PID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD);

    PID getRoll();
    PID getPitch();
    PID getYaw();

    PID getX();
    PID getY();
    PID getZ();

    void tuneAngle(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD);

    void tunePosition(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD);

    std::vector<double> update(std::vector<double> setPoint, std::vector<double> plantState, double dT);



};


#endif //CONTROLS_SIX_DEG_PID_H
