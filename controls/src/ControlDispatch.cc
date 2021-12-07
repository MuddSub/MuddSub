#include "controls/ControlDispatch.hh"

namespace MuddSub::Controls
{


ControlDispatch::ControlDispatch()
{
  controller_ = std::make_shared<SixDegreePID>();
  dynamics_.setController(controller_);
  controller_->setDynamics(std::shared_ptr<VehicleDynamics>(&dynamics_));

  setpointSub_ = nh_.subscribe("/robot_setpoint", 1, &ControlDispatch::setpointCallback, this);
  plantStateSub_ = nh_.subscribe("/slam/robot/state", 1, &ControlDispatch::plantCallback, this);
  resetSub_ = nh_.subscribe("/reset_controller", 1, &ControlDispatch::resetCallback, this);

  zeroSub_ = nh_.subscribe("/controls/robot/set_zero_to_odom", 1, &ControlDispatch::zeroToCallback, this);
  zeroHereSub_ = nh_.subscribe("/controls/robot/set_zero_here", 1, &ControlDispatch::zeroCurrentCallback, this);

  controlPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/controls/robot/wrench", 10);
}

stateVector_t ControlDispatch::odomToState(const nav_msgs::Odometry& msg)
{
  auto pos = msg.pose.pose.position;
  auto quatMsg = msg.pose.pose.orientation;

  double r,p,y;
  tf2::Quaternion quat;
  tf2::fromMsg(quatMsg, quat);
  tf2::Matrix3x3 m(quat);
  m.getRPY(r,p,y);

  auto vel = msg.twist.twist.linear;
  auto ang = msg.twist.twist.angular;

  stateVector_t state;

  state << pos.x, pos.y, pos.z, r, p, y, vel.x, vel.y, vel.z, ang.x, ang.y, ang.z;
  return state;
}

void ControlDispatch::plantCallback(const nav_msgs::Odometry& msg)
{
  auto state = odomToState(msg);

  plantState_ = state - plantZero_;

  controller_->setPlantState(plantState_);
  std::cout << " PLANT STATE: " << plantState_.format(eigenInLine) << std::endl;
}

void ControlDispatch::setpointCallback(const controls::State& state)
{
  const double* data = state.state.data();

  // ROS requires the parameter to function to be const, but eigen requires
  //   it not to be... so we must const_cast it away
  auto dataMutable = const_cast<double*>(data);

  auto stateVector = Eigen::Map<Eigen::Matrix<double, 12, 1>>(dataMutable);

  controller_->setSetpoint(stateVector);
  setpoint_ = stateVector;
}

void ControlDispatch::resetCallback(const std_msgs::Empty& msg)
{
  controller_->reset();
}

void ControlDispatch::publishControl(const controlVector_t& control)
{
  geometry_msgs::WrenchStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "alfie";

  msg.wrench.force.x = control[0];
  msg.wrench.force.y = control[1];
  msg.wrench.force.z = control[2];
  msg.wrench.torque.x = control[3];
  msg.wrench.torque.y = control[4];
  msg.wrench.torque.z = control[5];

  controlPub_.publish(msg);
}

void ControlDispatch::iterate()
{
  double t = ros::Time::now().toSec();
  controlVector_t control;
  controller_->computeControl(plantState_, t, control);

  broadcastStateAsTF(plantState_ + plantZero_, "world_ned", "robot_plant_state");
  broadcastStateAsTF(setpoint_ + plantZero_, "world_ned", "robot_setpoint");
  broadcastStateAsTF(plantZero_, "world_ned", "plant_zero");

  publishControl(control);
}

void ControlDispatch::setZero(const nav_msgs::Odometry& msg)
{
  plantZero_ = odomToState(msg);
}

void ControlDispatch::setZeroToCurrent()
{
  plantZero_ = plantState_ + plantZero_;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ControlsDispatch");

  MuddSub::Controls::ControlDispatch controller;

  ros::Rate loopRate = 20;
  while(ros::ok())
  {
    controller.iterate();
    ros::spinOnce();
    loopRate.sleep();
    std::cout << "\x1b[2J\x1b[H";
  }

}
