#pragma once

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "controls/ThrusterForceArray.h"
#include "controls/ThrusterPWMArray.h"

namespace MuddSub::Controls
{
  /**
   * @brief Publishes controls messages describing the robot's desired motion and the required thruster commands.
   *
   * The target sum of forces and torques on the robot as a whole at any given point in time are described by geometry_msgs/WrenchStampes messages published to the controls/robot/wrench topic.
   *
   * The target force outputs of each thruster required to attain the target sum of forces and torques are described by controls/ThrusterForceArray messages published to the controls/thruster/forces topic.
   *
   * The pwm signals required to attain each thruster's force output target are described by controls/ThrusterPWMArray messages published to the controls/thruster/pwms topic.
   */
  class ControlsPublisher
  {
    private:
      /**
       * @brief Publisher to controls/robot/wrench topic
       *
       * Publishes geometry_msgs/WrenchStamped messages
       */
      ros::Publisher wrench_pub;

      /**
       * @brief Publisher to controls/thruster/forces topic
       *
       * Publishes controls/ThrusterForceArray messages
       */
      ros::Publisher forces_pub;

      /**
       * @brief Publisher to controls/thruster/pwms topic
       *
       * Publishes controls/ThrusterPWMArray messages
       */
      ros::Publisher pwms_pub;

      /**
       * @brief Node handle
       *
       * NodeHandle is the main access point to communications with the ROS system.
       */
      ros::NodeHandle n_;

    public:
      /**
       * @brief Creates a ControlsPublisher instance
       *
       * @param n NodeHandle to create the Publishers with
       *
       * @return ControlsPublisher instance
       */
      ControlsPublisher(ros::NodeHandle n);

      /**
       * @brief Use default copy constructor
       */
      ControlsPublisher(const ControlsPublisher&) = default;

      /**
       * @brief Publishes a geometry_msgs/WrenchStamped message
       *
       * Uses the wrench_pub Publisher to publish to controls/robot/wrench
       *
       * @param wrenchStamped The WrenchStamped message to publish
       */
      void publishWrench(geometry_msgs::WrenchStamped& wrenchStamped);

      /**
       * @brief Publishes a controls/ThrusterForceArray message
       *
       * Uses the forces_pub Publisher to publish to controls/thruster/forces
       *
       * @param forces The ThrusterForceArray message to publish
       */
      void publishForces(controls::ThrusterForceArray& forces);

      /**
       * @brief Publishes a controls/ThrusterPWMArray message
       *
       * Uses the pwms_pub Publisher to publish to controls/thruster/pwms
       *
       * @param pwms The ThrusterPWMArray message to publish
       */
      void publishPWMs(controls::ThrusterPWMArray& pwms);
  };
} // namespace MuddSub::Controls
