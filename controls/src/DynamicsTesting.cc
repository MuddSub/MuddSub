#include "controls/VehicleDynamics.hh"

using stateVector_t = MuddSub::Controls::VehicleDynamics::stateVector_t;
using controlVector_t = MuddSub::Controls::VehicleDynamics::controlVector_t;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestDynamics");
  MuddSub::Controls::VehicleDynamics dynamics;

  stateVector_t x = stateVector_t::Zero();
  controlVector_t u = controlVector_t::Zero();
  stateVector_t xD = stateVector_t::Zero();


  u <<  0.3922,
        0.6555,
        0.1712,
        0.7060,
        0.0318,
        0.2769;

  x <<  0.1419,
        0.4218,
        0.9157,
        0.7922,
        0.9595,
        0.6557,
        0.0357,
        0.8491,
        0.9340,
        0.6787,
        0.7577,
        0.7431;

  dynamics.computeControlledDynamics(x, 0, u, xD);

  std::cout << xD << std::endl;
}
