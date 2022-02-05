#pragma once

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Odometry.h"
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
      /// @brief Publishes geometry_msgs/WrenchStamped messages to controls/robot/wrench topic
      ros::Publisher wrenchPub_;

      /// @brief Publishes nav_msgs/Odometry messages to controls/robot/error topic
      ros::Publisher errorPub_;

      /// @brief Publishes controls/ThrusterForceArray messages to controls/thruster/forces topic
      ros::Publisher forcesPub_;

      /// @brief Publishes controls/ThrusterPWMArray messages to controls/thruster/pwms topic
      ros::Publisher pwmsPub_;

      /// @brief Node handle, the main access point to communications with the ROS system.
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

      /// @brief Explicitly deleted default constructor
      ControlsPublisher() = delete;

      /// @brief Use default copy constructor
      ControlsPublisher(const ControlsPublisher&) = default;

      /**
       * @brief Publishes a geometry_msgs/WrenchStamped message
       *
       * Uses the wrenchPub_ Publisher to publish to controls/robot/wrench
       *
       * @param wrenchStamped The WrenchStamped message to publish
       */
      void publishWrench(geometry_msgs::WrenchStamped& wrenchStamped);

      /**
       * @brief Publishes a nav_msgs/Odometry message
       *
       * Uses the errorPub_ Publisher to publish to controls/robot/error
       *
       * @param odometry The Odometry message to publish
       */
      void publishError(nav_msgs::Odometry& odometry);

      /**
       * @brief Publishes a controls/ThrusterForceArray message
       *
       * Uses the forcesPub_ Publisher to publish to controls/thruster/forces
       *
       * @param forces The ThrusterForceArray message to publish
       */
      void publishForces(controls::ThrusterForceArray& forces);

      /**
       * @brief Publishes a controls/ThrusterPWMArray message
       *
       * Uses the pwmsPub_ Publisher to publish to controls/thruster/pwms
       *
       * @param pwms The ThrusterPWMArray message to publish
       */
      void publishPWMs(controls::ThrusterPWMArray& pwms);
  };
} // namespace MuddSub::Controls
