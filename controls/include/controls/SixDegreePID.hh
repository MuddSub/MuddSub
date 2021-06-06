#pragma once

#include "controls/Controller.hh"
#include "controls/Types.hh"
#include "controls/PID.hh"
#include <vector>
#include <array>

namespace MuddSub::Controls
{

class SixDegreePID : public Controller
{

private:
    PID roll_ = PID();
    PID pitch_ = PID();
    PID yaw_ = PID();

    PID surge_ = PID();
    PID sway_ = PID();
    PID heave_ = PID();

    std::array<PID, 6> controllers_{surge_, sway_, heave_, roll_, pitch_, yaw_};

public:
    SixDegreePID();
    SixDegreePID(std::vector<double> kP, std::vector<double> kI, std::vector<double> kD);

    inline void setSetpoint(const stateVector_t& setpoint)
    {
      setpoint_ = setpoint;
      reset();
    };

    PID& getRoll();
    PID& getPitch();
    PID& getYaw();

    PID& getSurge();
    PID& getSway();
    PID& getHeave();


    void tuneController(double kP, double kI, double kD,  PID& controller);

    //format of error is <surge, sway, heave, roll, pitch, yaw >
    void update(const stateVector_t& error,
                double dT,
                controlVector_t& control);

    void computeControl(const stateVector_t& state,
                        const double& t,
                        controlVector_t& controlAction);

    void reset()
    {
      for(auto& c : controllers_)
        c.resetController();
    };
};

}
