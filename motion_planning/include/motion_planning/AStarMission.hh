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

            void addMotion();
            void convertPath();
            std::vector<std::vector<double>> convertPoints(std::vector<geometry_msgs::Point> points);


        public:
            static geometry_msgs::PoseStamped getPoseStamped(std::vector<double> v);
            static std::vector<double> getPoseVector(geometry_msgs::PoseStamped poseStamped);

            static geometry_msgs::Point getPoint(std::vector<double> v);
            static std::vector<double> getPointVector(geometry_msgs::Point point);

            static std::vector<double> eulerToQuaternion (std::vector<double> e);
            static std::vector<double> quaternionToEuler (std::vector<double> q);

            std::map<std::string, std::vector<double>> targets_;
            std::vector<geometry_msgs::PoseStamped> finalPath_;
            std::vector<geometry_msgs::Point> obstacles_;


            AStarMission(std::vector<std::vector<std::string>> &goals, double velocity, int width, int height, int depth);
            AStarMission();
            void addGoals(std::vector<std::vector<std::string>> goals);

            void isSucessful(bool success);
            void recurse(geometry_msgs::PoseStamped poseStamped);
            void printPath();


    };
}