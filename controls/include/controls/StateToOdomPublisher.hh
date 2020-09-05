#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace MuddSub::Controls
{

/** @brief transforms 12-vector state to nav_msgs/Odometry message
 
A simple header-only class to take a 12-vector state and publish
it as a nav_msgs/Odometry message.
*/
class StateToOdomPublisher
{
public:
  StateToOdomPublisher() = delete;

  /// Set the advertised topic to topic, and adjust the publisher's buffer
  StateToOdomPublisher(const std::string& topic, size_t bufferSize = 10):
    topic_{topic}
  {
    odomPub_ = nh_.advertise<nav_msgs::Odometry>(topic_, bufferSize);
  }

  /// Convert and publish a 12-vector
  /// @param state: the 12DOF state to be turned into an odometry message and published
  void operator()( Eigen::Matrix<double, 12, 1>& state)
  {
    nav_msgs::Odometry msg;

    // Pack the header
    msg.header.seq = seq_;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "BASE";
    ++seq_;

    // Pose
    geometry_msgs::Pose& pose = msg.pose.pose;
    pose.position.x = state[0];
    pose.position.y = state[1];
    pose.position.z = state[2];

    tf2::Quaternion quat;
    quat.setRPY(state[3], state[4], state[5]);
    quat.normalize();
    pose.orientation = tf2::toMsg(quat);

    // Twist
    geometry_msgs::Twist& twist = msg.twist.twist;
    twist.linear.x = state[6];
    twist.linear.y = state[7];
    twist.linear.z = state[8];

    twist.angular.x = state[9];
    twist.angular.y = state[10];
    twist.angular.z = state[11];

    odomPub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;

  /// Publisher for the resulting odometry message
  ros::Publisher odomPub_;

  /// Topic name to publish to
  std::string topic_;

  /// Monotonically increasing ID of messages used for the header.
  size_t seq_{0};
};

}
