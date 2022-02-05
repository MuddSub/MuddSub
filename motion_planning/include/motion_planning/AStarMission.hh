#pragma once
#include "motion_planning/AStar.hh"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"





using MuddSub::MotionPlanning::AStar;


namespace MuddSub::MotionPlanning
{
    class AStarMission
    {
        private:
            
            std::vector<std::vector<std::string>> goals_;
            std::vector<geometry_msgs::Point> pastObstacles_;
            double velocity_;
            std::vector<double> pose_;
            AStar* current_;
            int width_;
            int height_;
            int depth_;

            std::vector<double> getPose(std::string poseString);


            std::vector<std::vector<double>> path_;

            void addMotion(std::vector<double> pose);
            void convertPath();
            std::vector<std::vector<double>> convertPoints(std::vector<geometry_msgs::Point> points);


        public:

            /**
            * Helper functions to convert between poseStamped, vectors (pose and a time), and a point
            **/
            static geometry_msgs::PoseStamped getPoseStamped(std::vector<double> v);
            static std::vector<double> getPoseVector(geometry_msgs::PoseStamped poseStamped);

            static geometry_msgs::Point getPoint(std::vector<double> v);
            static std::vector<double> getPointVector(geometry_msgs::Point point);

            /**
            * Switch between eulers and quaternions 
            **/
            static std::vector<double> eulerToQuaternion (std::vector<double> e);
            static std::vector<double> quaternionToEuler (std::vector<double> q);

            
            /**
            * @brief Can update targets based on the name and pose of the targets 
            **/
            std::map<std::string, std::vector<double>> targets_;

            /**
            * @brief this is the final path is what AStar returns
            **/
            std::vector<geometry_msgs::PoseStamped> finalPath_;

            /**
            * @brief Update this variable at any time to enter obstacles to the map
            **/
            std::vector<geometry_msgs::Point> obstacles_;

            /** 
            * Constructor
            * @param goals: add goals according to goal style shown below
            * @param velocity: velocity at which the robot goes at
            * @param width: width of the map
            * @param height: height of the map
            * @param depth: depth of the map
            **/
            AStarMission(std::vector<std::vector<std::string>> &goals, double velocity, int width, int height, int depth);

            /** 
            * @brief Default Constructor
            **/
            AStarMission();

            /**
            * @param goals: each vector in goals is of size three, with "goal", "linear movement", and "rotational movement", data needed
            * ex: ["1 1 1 0 pi/2 pi", "goToLocation", "None", ""]
            * ex: ["buoy_fairy", "goToTarget", "SineTraversalMovement", "10 10 0"]
            **/
            void addGoals(std::vector<std::vector<std::string>> goals);

            
            /**
            * @brief Goes to the next mission if the current mission is completed 
            * @param success: true if the mission is successful and false if not
            **/
            void isSucessful(bool success);

            /**
            * @param poseStamped This is a pose with a time that updates the path
            **/
            void recurse(geometry_msgs::PoseStamped poseStamped);

            /**
            * @brief prints the path to view it 
            **/
            void printPath();


    };
}