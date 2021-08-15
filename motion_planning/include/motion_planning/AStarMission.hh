#pragma once
#include "motion_planning/AStar.hh"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"



using MuddSub::MotionPlanning::AStar;


namespace MuddSub::MotionPlanning
{
    class AStarMission
    {
        private:
            std::vector<std::vector<std::string>> goals_;
            std::vector<std::vector<double>> pastObstacles_;
            double velocity_;
            std::vector<double> pose_;
            AStar* current_;
            int width_;
            int height_;
            int depth_;

            std::vector<double> getPose(std::string poseString);
            void addMotion();

            std::vector<double> getVector(geometry_msgs::PoseStamped poseStamped);
            geometry_msgs::PoseStamped getPoseStamped(std::vector<double> v);

        public:

            static std::vector<double> eulerToQuaternion (std::vector<double> e);
            static std::vector<double> quaternionToEuler (std::vector<double> q);

            std::map<std::string, std::vector<double>> targets_;
            std::vector<std::vector<double>> path_;
            std::vector<geometry_msgs::PoseStamped> finalPath_;
            std::vector<std::vector<double>> obstacles_;


            AStarMission(std::vector<std::vector<std::string>> &goals, double velocity, int width, int height, int depth);
            AStarMission();
            void addGoals(std::vector<std::vector<std::string>> goals);

            void isSucessful(bool success);
            void recurse(std::vector<double> pose, double time);
            void printPath();


    };
}