#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_planning/CreateTrajectoryAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "motion_planning/Trajectory.h"
#include "mission/RotationalMovement.h"
#include "mission/SinTraversalPath.h"
#include "slam/Map.h"
#include <iostream>
#include <vector>
#include <unordered_set>
#include "geometry_msgs/Point.h"
#include <math.h>
using MuddSub::MotionPlanning::AStar;
using MuddSub::MotionPlanning::AStarMission;
// maybe we also need to import slam/obstacles?
class CreateTrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<motion_planning::CreateTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  motion_planning::CreateTrajectoryFeedback feedback_;
  motion_planning::CreateTrajectoryResult result_;

public:

  CreateTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&CreateTrajectoryAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~CreateTrajectoryAction(void)
  {
  }

  void executeCB(const motion_planning::CreateTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    // feedback_.sequence.clear();
    // feedback_.sequence.push_back(0);
    // feedback_.sequence.push_back(1);

    // publish info to the console for the user
    // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("Dummy started");
    // start executing the action
    if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }

    // for(int i=1; i<=goal->order; i++)
    // {
    //   // check that preempt has not been requested by the client
    //   if (as_.isPreemptRequested() || !ros::ok())
    //   {
    //     ROS_INFO("%s: Preempted", action_name_.c_str());
    //     // set the action state to preempted
    //     as_.setPreempted();
    //     success = false;
    //     break;
    //   }
    //   feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
    //   // publish the feedback
    //   as_.publishFeedback(feedback_);
    //   // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    //   r.sleep();
    // }

    if(success)
    {
      geometry_msgs::PoseStamped poseStamped1;
      // ros::Time time();
      poseStamped.header.stamp = time;
      poseStamped.pose.position.x = 1.2;
      poseStamped.pose.position.y = 1.2;
      poseStamped.pose.position.z = 1.2;
      poseStamped.pose.orientation.x = 1.2;
      poseStamped.pose.orientation.y = 1.2;
      poseStamped.pose.orientation.z = 1.2;
      poseStamped.pose.orientation.w = 1.2;

      geometry_msgs::PoseStamped poseStamped2;
      // ros::Time time(v[6]);
      poseStamped.header.stamp = time;
      poseStamped.pose.position.x = 2.3;
      poseStamped.pose.position.y = 2.3;
      poseStamped.pose.position.z = 2.3;

      poseStamped.pose.orientation.x = 2.3;
      poseStamped.pose.orientation.y = 2.3;
      poseStamped.pose.orientation.z = 2.3;
      poseStamped.pose.orientation.w = 2.3;

      geometry_msgs::PoseStamped trajectory [2] = {poseStamped1, poseStamped2};
      result_.trajectory = trajectory;


      // result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");

  CreateTrajectoryAction trajectory("trajectory");
  ros::spin();

  return 0;
}