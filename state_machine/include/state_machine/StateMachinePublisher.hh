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
      /**
       * @brief Publisher to state_machine/objective topic
       *
       * Publishes actionlib_msgs/GoalID messages
       */
      ros::Publisher objective_pub;

      /**
       * @brief Publisher to state_machine/gripper topic
       *
       * Publishes state_machine/Gripper messages
       */
      ros::Publisher gripper_pub;

      /**
       * @brief Publisher to state_machine/torpedo topic
       *
       * Publishes state_machine/Torpedo messages
       */
      ros::Publisher torpedo_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
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

      /**
       * @brief Use default copy constructor
       */
      StateMachinePublisher(const StateMachinePublisher&) = default;

      /**
       * @brief Publishes an actionlib_msgs/GoalID message
       *
       * Uses the objective_pub Publisher to publish to state_machine/objective
       *
       * @param objective The GoalID message to publish
       */
      void publishObjective(actionlib_msgs::GoalID& objective);

      /**
       * @brief Publishes a state_machine/Gripper message
       *
       * Uses the gripper_pub Publisher to publish to state_machine/gripper
       *
       * @param gripper_command The Gripper message to publish
       */
      void publishGripperCommand(state_machine::Gripper& gripper_command);

      /**
       * @brief Publishes a state_machine/Torpedo message
       *
       * Uses the torpedo_pub Publisher to publish to state_machine/torpedo
       *
       * @param torpedo_command The Torpedo message to publish
       */
      void publishTorpedoCommand(state_machine::Torpedo& torpedo_command);
  };
} // namespace MuddSub::StateMachine
