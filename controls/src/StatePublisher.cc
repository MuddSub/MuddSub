#include "controls/StateToOdomPublisher.hh"
#include "controls/State.h"

MuddSub::Controls::StateToOdomPublisher statePub("robot_state");


void stateCB(const controls::State& msg)
{
  const double* data = msg.state.data();
  auto dataMutable = const_cast<double*>(data);

  Eigen::Matrix<double, 12, 1> stateVector = Eigen::Map<Eigen::Matrix<double, 12, 1>>(dataMutable);
  statePub(stateVector);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SimulationDynamics");
  ros::NodeHandle nh;

  nh.subscribe("robot_state_raw", 1, stateCB);

}
