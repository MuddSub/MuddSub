#pragma once

#include "ros/ros.h"
#include "nav_msgs/Path.h"

namespace MuddSub::MotionPlanning
{
  class MotionPlanningPublisher
  {
    private:
      /**
       * @brief Publisher to motion_planning/trajectory topic
       *
       * Publishes nav_msgs/Path messages
       */
      ros::Publisher motion_planning_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
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

      /**
       * @brief Use default copy constructor
       */
      MotionPlanningPublisher(const MotionPlanningPublisher&) = default;

      /**
       * @brief Publishes a nav_msgs/Path message
       *
       * Uses the motion_planning_pub Publisher to publish to motion_planning/trajectory
       *
       * @param trajectory The Path message to publish
       */
      void publishTrajectory(nav_msgs::Path& trajectory);
  };
} // namespace MuddSub::MotionPlanning
