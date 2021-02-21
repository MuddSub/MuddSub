#include "drivers/DepthSensorPublisher.hh"

namespace MuddSub::DepthSensor
{
  DepthSensorPublisher::DepthSensorPublisher(ros::NodeHandle n):
    n_(n)
  {
    depth_pub = n_.advertise<drivers::Depth>("depth_sensor/depth", 1000);
    pressure_pub = n_.advertise<sensor_msgs::FluidPressure>("depth_sensor/pressure", 1000);
    temperature_pub = n_.advertise<sensor_msgs::Temperature>("depth_sensor/temperature", 1000);
  }

  void DepthSensorPublisher::publishDepth(drivers::Depth& depth)
  {
    depth_pub.publish(depth);
  }

  void DepthSensorPublisher::publishPressure(sensor_msgs::FluidPressure pressure)
  {
    pressure_pub.publish(pressure);
  }

  void DepthSensorPublisher::publishTemperature(sensor_msgs::Temperature temperature)
  {
    temperature_pub.publish(temperature);
  }
}
