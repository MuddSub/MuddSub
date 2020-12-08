//
// Created by tanvi on 10/30/2020.
//

#ifndef CONTROLS_PID_H
#define CONTROLS_PID_H


class PID
{
private:
    //double deltaT_;
    double kP_;
    double kI_;
    double kD_;
    bool angle_;

    double integralError_;
    double previousError_;

public:
    PID(double kP, double kI, double kD);
    PID(double kP, double kI, double kD, bool angle);
    PID();

    void tune(double kP, double kI, double kD);

    double getkI();
    double getkP();
    double getkD();

    double update(double setPoint, double plantState,  double deltaT);


};


#endif //CONTROLS_PID_H
