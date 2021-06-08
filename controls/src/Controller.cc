#include "controls/Controller.hh"

namespace MuddSub::Controls
{

Controller::Controller(): tf2Listener_{tfBuffer_}
{
}

stateVector_t Controller::getError()
{
  stateVector_t result = stateVector_t::Zero();

  // Transform the plantState into the frame of the robot
  // TODO: Put this in some utils
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform("robot_plant_state", "robot_setpoint",
                             ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    ROS_WARN("%s",e.what());
    ros::Duration(0.1).sleep();
    return result;
  }

  // Translation errors
  result[0] = transformStamped.transform.translation.x;
  result[1] = transformStamped.transform.translation.y;
  result[2] = transformStamped.transform.translation.z;

  // Roll, pitch, yaw errors
  tf2::Quaternion q;
  tf2::fromMsg(transformStamped.transform.rotation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(result[3], result[4], result[5]);

  for(int i = 3; i < 6; ++i)
  {
    double& val = result[i];
    if(val > M_PI)
    {
      val = M_PI - val;
    }
  }

  geometry_msgs::Vector3Stamped velocityError;
  velocityError.vector.x = setpoint_[6] - plantState_[6];
  velocityError.vector.y = setpoint_[7] - plantState_[7];
  velocityError.vector.z = setpoint_[8] - plantState_[8];

  geometry_msgs::Vector3Stamped angularVelocityError;
  velocityError.vector.x = setpoint_[9] - plantState_[9];
  velocityError.vector.y = setpoint_[10] - plantState_[10];
  velocityError.vector.z = setpoint_[11] - plantState_[11];

  geometry_msgs::TransformStamped plantTransformStamped;

  try
  {
    plantTransformStamped = tfBuffer_.lookupTransform("world_ned", "robot_plant_state",
                             ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    ROS_WARN("%s",e.what());
    ros::Duration(0.1).sleep();
    return result;
  }

  geometry_msgs::Vector3Stamped velocityErrorTransformed,
                                angularVelocityErrorTransformed;

  tf2::doTransform(velocityError, velocityErrorTransformed, plantTransformStamped);
  tf2::doTransform(angularVelocityError, angularVelocityErrorTransformed, plantTransformStamped);

  result[6] = velocityErrorTransformed.vector.x;
  result[7] = velocityErrorTransformed.vector.y;
  result[8] = velocityErrorTransformed.vector.z;

  result[9] = angularVelocityErrorTransformed.vector.x;
  result[10] = angularVelocityErrorTransformed.vector.y;
  result[11] = angularVelocityErrorTransformed.vector.z;

  return result;
}

double Controller::angleError(double setpoint, double plantState)
{
  double error{0};
  auto wrapAngle = [](double num){
    double mod = std::fmod(num, 2*M_PI);
    if(mod < 0)
      return mod + 2*M_PI;
    return mod;
  };

  setpoint = wrapAngle(setpoint);
  plantState = wrapAngle(plantState);

  error = std::fmod(setpoint - plantState, 2*M_PI);
  if(error < 0) error += 2*M_PI;

  double altError = wrapAngle(2*M_PI - error);

  double chosenError = std::min(error, altError);

  // Find the sign by adding to plantstate and seeing if it's right
  double sumError = wrapAngle(plantState + chosenError);
  double diffError = wrapAngle(plantState - chosenError);

  if(sumError < diffError)
    error = chosenError;
  else
    error = -1 * chosenError;

  return error;
}


} // namespace MuddSub::Controls
