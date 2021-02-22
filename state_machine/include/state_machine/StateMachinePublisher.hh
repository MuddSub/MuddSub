#pragma once

#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"
#include "state_machine/Gripper.h"
#include "state_machine/Torpedo.h"

namespace MuddSub::StateMachine
{
  class StateMachinePublisher
  {
    private:
      /// @brief Publisher to state_machine/objective topic
      ros::Publisher objectivePub_;

      /// @brief Publisher to state_machine/gripper topic
      ros::Publisher gripperPub_;

      /// @brief Publisher to state_machine/torpedo topic
      ros::Publisher torpedoPub_;

      /// @brief Node handle NodeHandle is the main access point to communications with the ROS system.
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a StateMachinePublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return StateMachinePublisher instance
       */
      StateMachinePublisher(ros::NodeHandle n);

      /// @brief Explecitly delete default constructor
      StateMachinePublisher() = delete;

      /// @brief Use default copy constructor
      StateMachinePublisher(const StateMachinePublisher&) = default;

      /**
       * @brief Publishes an actionlib_msgs/GoalID message
       *
       * Uses the objectivePub_ Publisher to publish to state_machine/objective
       *
       * @param objective The GoalID message to publish
       */
      void publishObjective(actionlib_msgs::GoalID& objective);

      /**
       * @brief Publishes a state_machine/Gripper message
       *
       * Uses the gripperPub_ Publisher to publish to state_machine/gripper
       *
       * @param gripperCommand The Gripper message to publish
       */
      void publishGripperCommand(state_machine::Gripper& gripperCommand);

      /**
       * @brief Publishes a state_machine/Torpedo message
       *
       * Uses the torpedoPub_ Publisher to publish to state_machine/torpedo
       *
       * @param torpedoCommand The Torpedo message to publish
       */
      void publishTorpedoCommand(state_machine::Torpedo& torpedoCommand);
  };
} // namespace MuddSub::StateMachine
