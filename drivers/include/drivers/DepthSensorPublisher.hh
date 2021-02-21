#pragma once

#include "ros/ros.h"
#include "drivers/Depth.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"

namespace MuddSub::DepthSensor
{
  /**
   * @brief Publishes depth sensor data.
   *
   * The sensed depth of the robot is published in depth_sensor/Depth messages to the depth_sensor/depth topic.
   *
   * The sensed fluid pressure is published in sensor_msgs/FluidPressure messages to the depth_sensor/pressure topic.
   *
   * The sensed temperature is published in sensor_msgs/Temperature messages to the depth_sensor/temperature topic.
   */
  class DepthSensorPublisher
  {
    private:
      /**
       * @brief Publisher to depth_sensor/depth topic
       *
       * Publishes depth_sensor/Depth messages
       */
      ros::Publisher depth_pub;

      /**
       * @brief Publisher to depth_sensor/pressure topic
       *
       * Publishes sensor_msgs/FluidPressure messages
       */
      ros::Publisher pressure_pub;

      /**
       * @brief Publisher to depth_sensor/temperature topic
       *
       * Publishes sensor_msgs/Temperature messages
       */
      ros::Publisher temperature_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a DepthSensorPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return DepthSensorPublisher instance
       */
      DepthSensorPublisher(ros::NodeHandle n);

      /**
       * @brief Use default copy constructor
       */
      DepthSensorPublisher(const DepthSensorPublisher&) = default;

      /**
       * @brief Publishes a depth_sensor/Depth message
       *
       * Uses the depth_pub Publisher to publish to depth_sensor/depth
       *
       * @param depth The Depth message to publish
       */
      void publishDepth(drivers::Depth& depth);

      /**
       * @brief Publishes a sensor_msgs/FluidPressure message
       *
       * Uses the pressure_pub Publisher to publish to depth_sensor/pressure
       *
       * @param pressure The FluidPressure message to publish
       */
      void publishPressure(sensor_msgs::FluidPressure pressure);

      /**
       * @brief Publishes a sensor_msgs/Temperature message
       *
       * Uses the temperature_pub Publisher to publish to depth_sensor/temperature
       *
       * @param temperature The Temperature message to publish
       */
      void publishTemperature(sensor_msgs::Temperature temperature);
  };
} // namespace MuddSub::DepthSensor
