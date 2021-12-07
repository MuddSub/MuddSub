namespace MuddSub::Controls
{

nav_msgs::Odometry stateToOdom(const stateVector_t& state)
{
  nav_msgs::Odometry result;

  // Position
  geometry_msgs::Point& position = result.pose.pose.position;

  position.x = state[0];
  position.y = state[1];
  position.z = state[2];

  // orientation
  tf2::Quaternion quat;
  quat.setRPY(state[3], state[4], state[5]);

  result.pose.pose.orientation = tf2::toMsg(quat);

  // Linear Velocity
  geometry_msgs::Vector3& linear = result.twist.twist.linear;

  linear.x = state[0];
  linear.y = state[1];
  linear.z = state[2];

  // Angular Velocity
  geometry_msgs::Vector3& angular = result.twist.twist.angular;

  angular.x = state[0];
  angular.y = state[1];
  angular.z = state[2];

  return result;
};

void broadcastStateAsTF(const stateVector_t& state,
                        const std::string& parentFrame,
                        const std::string& childFrame)
{
  nav_msgs::Odometry odom = stateToOdom(state);

  static tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = parentFrame;
  transform.child_frame_id = childFrame;

  auto& position = odom.pose.pose.position;
  auto& orientation = odom.pose.pose.orientation;

  transform.transform.translation.x = position.x;
  transform.transform.translation.y = position.y;
  transform.transform.translation.z = position.z;

  transform.transform.rotation = orientation;

  broadcaster.sendTransform(transform);
};



} // namespace MuddSub::Core::Utils
