#include "controls/StateToOdomPublisher.hh"
#include "controls/State.h"

namespace MuddSub::Controls
{
class StatePublisher
{
public:
  MuddSub::Controls::StateToOdomPublisher statePub_{"robot_state"};
  ros::NodeHandle nh_;
  ros::Subscriber stateSub_;
  void stateCB(const controls::State& msg)
  {
    const double* data = msg.state.data();
    auto dataMutable = const_cast<double*>(data);

    Eigen::Matrix<double, 12, 1> stateVector = Eigen::Map<Eigen::Matrix<double, 12, 1>>(dataMutable);
    statePub_(stateVector);
  };

  StatePublisher()
  {
    stateSub_ = nh_.subscribe("robot_state_raw", 5, &StatePublisher::stateCB, this);
  };
};
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "StatePublisher");

  MuddSub::Controls::StatePublisher statePub;

  ros::Rate loopRate = 30;
  while(ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }

}
