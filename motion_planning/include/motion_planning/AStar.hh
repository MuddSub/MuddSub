
#pragma once
//#include <src/AStar.cc>
#include <limits>
#include <math.h>
#include <ros/ros.h>

namespace MuddSub::MotionPlanning
{
class AStar
{
public:
    std::vector<std::vector<double>> path_;
    enum rotationalMotion_ {rotationalMovement, sinTraversalPath, None};
    AStar(int width, int height, int depth, double prob, int startx, int starty, int startz, int endx, int endy, int endz, rotationalMotion_ motion, std::string data);
    AStar(int width, int height, int depth);


private:
    class Node;
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
    //struct pose{ double *position;};
    Node*** grid_;

    void makeGrid();
    void printGrid();
    void solveGrid();
    void makeParent(Node *node);
    void solveGridDFS();
    void DFS(int x, int y, int z);
    void addRotation(std::string data);
    void addSinTraversal(std::string data);
    void addTime();

    rotationalMotion_ motion_;
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
        rotationalMotion_ motion_;

        Node(int x, int y, int z, Node *end);
        Node();
        void setObstacle(bool isObstacle);
        friend bool operator< (const Node& a, const Node& b){return a.f_ > b.f_;}
        friend bool operator> (const Node& a, const Node& b) {return a.f_ <b.f_;}

        template<typename T>
        bool operator()(T *a, T *b) {
            return a->f_ < b->f_;
        }
    };
}
