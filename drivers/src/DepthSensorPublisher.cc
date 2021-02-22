#include "drivers/DepthSensorPublisher.hh"

namespace MuddSub::DepthSensor
{
  DepthSensorPublisher::DepthSensorPublisher(ros::NodeHandle n):
    n_(n)
  {
    depthPub_ = n_.advertise<drivers::Depth>("drivers/depth_sensor/depth", 1000);
    pressurePub_ = n_.advertise<sensor_msgs::FluidPressure>("drivers/depth_sensor/pressure", 1000);
    temperaturePub_ = n_.advertise<sensor_msgs::Temperature>("drivers/depth_sensor/temperature", 1000);
  }

  void DepthSensorPublisher::publishDepth(drivers::Depth& depth)
  {
    depthPub_.publish(depth);
  }

  void DepthSensorPublisher::publishPressure(sensor_msgs::FluidPressure pressure)
  {
    pressurePub_.publish(pressure);
  }

  void DepthSensorPublisher::publishTemperature(sensor_msgs::Temperature temperature)
  {
    temperaturePub_.publish(temperature);
  }
}
