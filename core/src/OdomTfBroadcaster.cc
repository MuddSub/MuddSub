#include "core/OdomTfBroadcaster.hh"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

namespace MuddSub::Core
{
OdomTfBroadcaster::OdomTfBroadcaster(ros::NodeHandle nh,
                                     const std::string& odomTopic,
                                     const std::string parentFrame,
                                     const std::string& childFrame):
    topic_(odomTopic), parentFrame_(parentFrame), childFrame_(childFrame), nh_(nh)
{
  odomSub_ = nh_.subscribe(topic_, 1, &OdomTfBroadcaster::odomCallback, this);
}


void OdomTfBroadcaster::odomCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Callback");
  tf::Transform transform;
  geometry_msgs::Point p = msg.pose.pose.position;
  geometry_msgs::Quaternion q = msg.pose.pose.orientation;
  transform.setOrigin(tf::Vector3(p.x,p.y, p.z));

  tf::Quaternion quatTf(q.x, q.y, q.z, q.w);
  quatTf.normalize();
  transform.setRotation(quatTf);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parentFrame_, childFrame_));
}
}
