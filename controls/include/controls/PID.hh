#pragma once
#include <ros/ros.h>

namespace MuddSub::Controls
{

class PID
{
private:
    double kP_;
    double kI_;
    double kD_;
    bool isAngle_;

    double integralError_;
    double previousError_;

    double windupLimit_{100};

    std::string tuneParamRoot_;
    bool tuneFromParams_{false};

    ros::NodeHandle nh_;

    double derivativeError_;

public:
    PID(double kP, double kI, double kD);

    //isAngle true when PID controls an angle, and false when PID controls a position
    PID(double kP, double kI, double kD, bool isAngle);

    PID(double kP, double kI, double kD, bool isAngle, double windupLimit);

    PID(std::string tuneParamRoot);

    PID();

    void tune(double kP, double kI, double kD);

    double getKI();
    double getKP();
    double getKD();

    double update(double setPoint, double plantState, double deltaT);


};

}
