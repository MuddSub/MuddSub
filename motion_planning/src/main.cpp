#include <iostream>
#include <unordered_set>
#include "motion_planning/AStar.hh"
using MuddSub::MotionPlanning::AStar;

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
    int map1[1][3][3] = {{{0,1,1},{0,0,0},{0,0,0}}};

}

