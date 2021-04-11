
#pragma once
//#include <src/AStar.cc>
#include <limits>
#include <math.h>
#include <ros/ros.h>

namespace MuddSub::MotionPlanning
{
class AStar
{
private:
    double*** distance_;
    int width_;
    int height_;
    int depth_;
    double prob_;
    int startx_;
    int starty_;
    int startz_;
    int endx_;
    int endy_;
    int endz_;
    void makeGrid();
    void printGrid();
    void solveGrid();
    void solveGridDFS();
    void DFS(int x, int y, int z);

    std::string goalId_;
    std::string motion_style_;
    double velocity_;

    class Node
    {
    public:
        static double infinity;
        bool isOpen_ = false;
        bool isClosed_ = false;
        bool isObstacle_ = false;
        int x_;
        int y_;
        int z_;
        double f_ = infinity;
        double  g_ = infinity;
        double  h_ = 0;
        std::string str_  = "0";
        Node* parent_ = NULL;
        Node(int x, int y, int z, Node *end);
        Node();
        void setObstacle(bool isObstacle);
        friend bool operator< (const Node& a, const Node& b){return a.f_ > b.f_;}
        friend bool operator> (const Node& a, const Node& b) {return a.f_ > b.f_;}
        //operator std::string() const {return str_;}
    };
public:
    Node*** grid_;
    AStar(int width, int height, int depth, double prob, int startx, int starty, int startz, int endx, int endy, int endz);
    AStar(int width, int height, int depth);
    void makeParent(Node *node);
    //double static distance (int x1, int y1, int z1, int x2, int y2, int z2);

    void goToTarget(std::string landmark_name, double velocity, std::string motion_style);
};
}
