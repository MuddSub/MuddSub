#pragma once

#include "ros/ros.h"
#include "nav_msgs/Path.h"

namespace MuddSub::MotionPlanning
{
  class MotionPlanningPublisher
  {
    private:
      /// @brief Publishes nav_msgs/Path messages to motion_planning/trajectory topic
      ros::Publisher pathPub_;

      /// @brief Node handle, the main access point to communications with the ROS system.
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a MotionPlanningPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return MotionPlanningPublisher instance
       */
      MotionPlanningPublisher(ros::NodeHandle n);

      /// @brief Explicitly deleted default constructor
      MotionPlanningPublisher() = delete;

      /// @brief Use default copy constructor
      MotionPlanningPublisher(const MotionPlanningPublisher&) = default;

      /**
       * @brief Publishes a nav_msgs/Path message
       *
       * Uses the pathPub_ Publisher to publish to motion_planning/trajectory
       *
       * @param trajectory The Path message to publish
       */
      void publishTrajectory(nav_msgs::Path& trajectory);
  };
} // namespace MuddSub::MotionPlanning
