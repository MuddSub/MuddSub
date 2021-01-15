#include "controls/ControlDispatch.hh"

namespace MuddSub::Controls
{


ControlDispatch::ControlDispatch()
{
  controller_ = std::make_shared<DecoupledLQR>();
  dynamics_.setController(controller_);
  controller_->setDynamics(std::shared_ptr<VehicleDynamics>(&dynamics_));

  setpointSub_ = nh_.subscribe("robot_setpoint", 1, &ControlDispatch::setpointCallback, this);
  plantStateSub_ = nh_.subscribe("slam/robot/pose", 1, &ControlDispatch::plantCallback, this);

  controlPub_ = nh_.advertise<geometry_msgs::Wrench>("/controls/robot/wrench", 10);
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
  plantState_ = odomToState(msg);
  ROS_INFO("Updated plant state");
}

void ControlDispatch::setpointCallback(const controls::State& state)
{
  const double* data = state.state.data();

  // ROS requires the parameter to function to be const, but eigen requires
  //   it not to be... so we must const_cast it away
  auto dataMutable = const_cast<double*>(data);

  auto stateVector = Eigen::Map<Eigen::Matrix<double, 12, 1>>(dataMutable);
  controller_->setSetpoint(stateVector);
  ROS_INFO("Updated setpoint");
}

void ControlDispatch::publishControl(const controlVector_t& control)
{
  geometry_msgs::Wrench msg;

  msg.force.x = control[0];
  msg.force.y = control[1];
  msg.force.z = control[2];
  msg.torque.x = control[3];
  msg.torque.y = control[4];
  msg.torque.z = control[5];

  controlPub_.publish(msg);
}

void ControlDispatch::iterate()
{
  double t = ros::Time::now().toSec();
  controlVector_t control;
  controller_->computeControl(plantState_, t, control);

  publishControl(control);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ControlsDispatch");
  ros::NodeHandle nh;

  MuddSub::Controls::ControlDispatch controller;

  ros::Rate loopRate = 20;
  while(ros::ok())
  {
    controller.iterate();
    ros::spinOnce();
    loopRate.sleep();
  }

}
