#include "controls/ControlsPublisher.hh"

namespace MuddSub::Controls
{
  ControlsPublisher::ControlsPublisher(ros::NodeHandle n):
    n_(n)
  {
    wrench_pub = n_.advertise<geometry_msgs::WrenchStamped>("controls/robot/wrench", 1000);
    forces_pub = n_.advertise<controls::ThrusterForceArray>("controls/thruster/forces", 1000);
    pwms_pub = n_.advertise<controls::ThrusterForceArray>("controls/thruster/pwms", 1000);
  }

  void ControlsPublisher::publishWrench(geometry_msgs::WrenchStamped& wrenchStamped)
  {
    wrench_pub.publish(wrenchStamped);
  }

  void ControlsPublisher::publishForces(controls::ThrusterForceArray& forces)
  {
    forces_pub.publish(forces);
  }

  void ControlsPublisher::publishPWMs(controls::ThrusterPWMArray& pwms)
  {
    pwms_pub.publish(pwms);
  }
}
