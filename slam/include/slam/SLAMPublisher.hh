#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "slam/Map.h"

namespace MuddSub::SLAM
{
  class SLAMPublisher
  {
    private:
      /**
       * @brief Publisher to slam/robot/state topic
       *
       * Publishes nav_msgs/Odometry messages
       */
      ros::Publisher state_pub;

      /**
       * @brief Publisher to slam/map topic
       *
       * Publishes slam/Map messages
       */
      ros::Publisher map_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a SLAMPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return SLAMPublisher instance
       */
      SLAMPublisher(ros::NodeHandle n);

      /**
       * @brief Use default copy constructor
       */
      SLAMPublisher(const SLAMPublisher&) = default;

      /**
       * @brief Publishes a nav_msgs/Odometry message
       *
       * Uses the state_pub Publisher to publish to slam/robot/state
       *
       * @param odometry The Odometry message to publish
       */
      void publishState(nav_msgs::Odometry& odometry);

      /**
       * @brief Publishes a slam/Map message
       *
       * Uses the map_pub Publisher to publish to slam/map
       *
       * @param map The Map message to publish
       */
      void publishMap(slam::Map& map);
  };
} // namespace MuddSub::SLAM
