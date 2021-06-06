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
    transformStamped = tfBuffer_.lookupTransform("turtle2", "turtle1",
                             ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    ROS_WARN("%s",e.what());
    ros::Duration(0.1).sleep();
    return result;
  }

  auto position = geometry_msgs::Vector3Stamped;
  position.vector.x = plantState[0];
  position.vector.y = plantState[1];
  position.vector.z = plantState[2];

  auto velocity = geometry_msgs::Vector3Stamped;
  position.vector.x = plantState[6];
  position.vector.y = plantState[7];
  position.vector.z = plantState[8];


  for(auto i = 0; i < 12; ++i)
  {
    double setpointElement{setpoint_[i]};
    double plantStateElement{plantState[i]};
    double error{0};

    // Elements 3-5 are angles
    if(i >= 3 && i <= 5)
    {
      error = angleError(setpointElement, plantStateElement);
    }
    else
    {
      error = setpointElement - plantStateElement;
    }

    result[i] = error;
  }
  std::cerr << "ERROR: " << result << std::endl;
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
