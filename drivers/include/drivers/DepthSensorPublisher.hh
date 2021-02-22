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
   * The sensed depth of the robot is published in drivers/depth_sensor/Depth messages to the drivers/depth_sensor/depth topic.
   *
   * The sensed fluid pressure is published in sensor_msgs/FluidPressure messages to the drivers/depth_sensor/pressure topic.
   *
   * The sensed temperature is published in sensor_msgs/Temperature messages to the drivers/depth_sensor/temperature topic.
   */
  class DepthSensorPublisher
  {
    private:
      /**
       * @brief Publisher to drivers/depth_sensor/depth topic
       *
       * Publishes drivers/depth_sensor/Depth messages
       */
      ros::Publisher depthPub_;

      /// @brief Publishes sensor_msgs/FluidPressure messages to drivers/depth_sensor/pressure topic
      ros::Publisher pressurePub_;

      /// @brief Publishes sensor_msgs/Temperature messages to drivers/depth_sensor/temperature topic
      ros::Publisher temperaturePub_;

      /// @brief Node handle, the main access point to communications with the ROS system.
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

      /// @brief Explicitly deleted default constructor
      DepthSensorPublisher() = delete;

      /// @brief Use default copy constructor
      DepthSensorPublisher(const DepthSensorPublisher&) = default;

      /**
       * @brief Publishes a drivers/depth_sensor/Depth message
       *
       * Uses the depthPub_ Publisher to publish to drivers/depth_sensor/depth
       *
       * @param depth The Depth message to publish
       */
      void publishDepth(drivers::Depth& depth);

      /**
       * @brief Publishes a sensor_msgs/FluidPressure message
       *
       * Uses the pressurePub_ Publisher to publish to drivers/depth_sensor/pressure
       *
       * @param pressure The FluidPressure message to publish
       */
      void publishPressure(sensor_msgs::FluidPressure pressure);

      /**
       * @brief Publishes a sensor_msgs/Temperature message
       *
       * Uses the temperaturePub_ Publisher to publish to drivers/depth_sensor/temperature
       *
       * @param temperature The Temperature message to publish
       */
      void publishTemperature(sensor_msgs::Temperature temperature);
  };
} // namespace MuddSub::DepthSensor
