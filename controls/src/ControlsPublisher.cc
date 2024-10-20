#include "controls/ControlsPublisher.hh"

namespace MuddSub::Controls
{
  ControlsPublisher::ControlsPublisher(ros::NodeHandle n):
    n_(n)
  {
    wrenchPub_ = n_.advertise<geometry_msgs::WrenchStamped>("controls/robot/wrench", 1000);
    forcesPub_ = n_.advertise<controls::ThrusterForceArray>("controls/thruster/forces", 1000);
    pwmsPub_ = n_.advertise<controls::ThrusterForceArray>("controls/thruster/pwms", 1000);
  }

  void ControlsPublisher::publishWrench(geometry_msgs::WrenchStamped& wrenchStamped)
  {
    wrenchPub_.publish(wrenchStamped);
  }

  void ControlsPublisher::publishForces(controls::ThrusterForceArray& forces)
  {
    forcesPub_.publish(forces);
  }

  void ControlsPublisher::publishPWMs(controls::ThrusterPWMArray& pwms)
  {
    pwmsPub_.publish(pwms);
  }
}
