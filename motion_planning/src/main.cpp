#include <iostream>
#include <vector>
#include <unordered_set>
#include "motion_planning/AStar.hh"
#include "motion_planning/AStarMission.hh"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
using MuddSub::MotionPlanning::AStar;
using MuddSub::MotionPlanning::AStarMission;

int main()
{
    std::cout<<"hello"<<std::endl;
    std::unordered_set<int> hello;
    hello.insert(0);

    AStar* pStar = new AStar(1,15,15);
    pStar->solveGrid();
    pStar->solveGridDFS();
    pStar->printGrid();
    pStar->updatePose(0,3,2);
    pStar->printGrid();
    pStar->updatePose(0,9,2);
    pStar->printGrid();
    pStar->updatePose(0,13,3);
    pStar->printGrid();
    pStar->updatePose(0,5,10);
    pStar->printGrid();
    std::vector<std::vector<double>> path = pStar->path_;
    for(int i = 0; i < path.size(); i++)
    {
        std::cout<<path[i][0]<<' '<<path[i][1]<<' '<<path[i][2]<<std::endl;
    }
    
    //Testing Mission Interface 
    int map1[1][3][3] = {{{0,1,1},{0,0,0},{0,0,0}}};

    std::vector<std::string> goal1 = {"0 1 7 0.5 0.6 0.7", "goToLocation", "None", ""};
    std::vector<std::string> goal2 = {"0 2 2 0.5 0.5 0.5", "SineTraversalMovement", "2 2 0"};
    std::vector<std::vector<std::string>> goals;
    goals.push_back(goal1);
    goals.push_back(goal2);
    
    AStarMission* mission = new AStarMission(goals, 5, 1, 10,10);

    std::vector<double> obstacle1 = {0, 5, 5, 0.1, 0.1, 0};
    std::vector<double> obstacle2 = {0, 1, 1, 0, 0, 0};
    std::vector<std::vector<double>> obstacles;
    obstacles.push_back(obstacle1);
    obstacles.push_back(obstacle2);

    mission->obstacles_ = obstacles;

    std::vector<double> currentPose = {0, 0, 0, 0, 0, 0};
    mission->recurse(currentPose, 0);
    mission->printPath();

    std::vector<double> obstacle3 = {0,0,2};
    obstacles.push_back(obstacle3);
    mission->obstacles_ = obstacles;
    mission->recurse(currentPose, 0.5);
    mission->printPath();

    mission->isSucessful(true);
    currentPose[2] = 3;

    mission->recurse(currentPose, 1);
    mission->printPath();


    //Testing DataString
    /*std::string data = "hello hi how are you doing today !";
    std:: vector<std::string> split = AStar::splitData(data);
    for(int i = 0; i < split.size(); i++)
    {
        std::cout<<i<<' '<<split[i]<<std::endl;
    }*/

    //Testing Pose Stamped
    geometry_msgs::PoseStamped poseStamped;
    ros::Time time(5);
    poseStamped.header.stamp = time;
    poseStamped.pose.position.x = 5;
    double y_pos = 5;
    poseStamped.pose.position.y = y_pos;

    std::cout<<poseStamped.pose.position.y<<std::endl;
    std::cout<<cos(0)<<std::endl;



    //testing q -> e, and e -> q
    std::vector<double> euler;
    euler.push_back(0.3);
    euler.push_back(0.4);
    euler.push_back(0.5);

    std::vector<double> q = AStarMission::eulerToQuaternion(euler);
    std::vector<double> e2 = AStarMission::quaternionToEuler(q);

    std::cout<<e2[0]<<e2[1]<<e2[2]<<std::endl;



}

