#include "controls/SimulationDynamics.hh"

namespace MuddSub::Controls
{

SimulationDynamics::SimulationDynamics(): statePub_("robot_state")
{
  ROS_INFO("Construct");
  controller_ = std::make_shared<DecoupledLQR>();
  ROS_INFO("Controller constructed");
  dynamics_.setController(controller_);
  controller_->setDynamics(std::shared_ptr<VehicleDynamics>(&dynamics_));
  setpointSub_ = nh_.subscribe("robot_setpoint", 1, &SimulationDynamics::setpointCB, this);
}

// Subscribe to the setpoint (currently a 12-vector for convenience, will
// later switch to a geometry_msgs/Pose)
void SimulationDynamics::setpointCB(const controls::State& state)
{
  const double* data = state.state.data();

  // ROS requires the parameter to function to be const, but eigen requires
  //   it not to be... so we must const_cast it away
  auto dataMutable = const_cast<double*>(data);

  auto stateVector = Eigen::Map<Eigen::Matrix<double, 12, 1>>(dataMutable);
  controller_->setSetpoint(stateVector);
}

void SimulationDynamics::runOnce()
{

  // Propogate previous state derivative
  double newTime = ros::Time::now().toSec();
  double deltaT = newTime - prevTime_;
  prevTime_ = newTime;
  state_ += deltaT * stateDerivative_;
  statePub_(state_);

  // Compute new state derivative
  dynamics_.computeDynamics(state_, newTime, stateDerivative_);
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SimulationDynamics");
  ros::NodeHandle nh;
  MuddSub::Controls::SimulationDynamics simulation;

  ros::Rate loopRate = 5;
  double t = ros::Time::now().toSec();
  while(ros::ok())
  {
    simulation.runOnce();
    double newT = ros::Time::now().toSec();
    t = newT;
    loopRate.sleep();
    ros::spinOnce();
  }
}
