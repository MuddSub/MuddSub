#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "slam/Map.h"

namespace MuddSub::SLAM
{
  /**
   * @brief Publishes SLAM messages describing the states of both the robot and the environment.
   *
   * The state of the robot, composed of its coordinates, velocity, and uncertainties are represented by nav_msgs/Odometry messages published to the slam/robot/state topic.
   *
   * The map of the environment, composed of landmark coordinates and uncertainties are represented by slam/Map messages published to the slam/map topic.
   */
  class SLAMPublisher
  {
    private:
      /// @brief Publishes nav_msgs/Odometry messages to slam/robot/state topic
      ros::Publisher statePub_;

      /// @brief Publishes slam/Map messages to slam/map topic
      ros::Publisher mapPub_;

      /// @brief Node handle, the main access point to communications with the ROS system.
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

      /// @brief Explicitly deleted default constructor
      SLAMPublisher() = delete;

      /// @brief Use default copy constructor
      SLAMPublisher(const SLAMPublisher&) = default;

      /**
       * @brief Publishes a nav_msgs/Odometry message
       *
       * Uses the statePub_ Publisher to publish to slam/robot/state
       *
       * @param odometry The Odometry message to publish
       */
      void publishState(nav_msgs::Odometry& odometry);

      /**
       * @brief Publishes a slam/Map message
       *
       * Uses the mapPub_ Publisher to publish to slam/map
       *
       * @param map The Map message to publish
       */
      void publishMap(slam::Map& map);
  };
} // namespace MuddSub::SLAM
