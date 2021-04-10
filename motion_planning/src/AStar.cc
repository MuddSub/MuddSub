

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <queue>
#include "motion_planning/AStar.hh"
namespace MuddSub::MotionPlanning
{

double  AStar::Node::infinity   = std::numeric_limits<double>::infinity();
AStar::AStar(int width, int height, int depth, double prob, int startx, int starty, int startz, int endx, int endy,
             int endz)
{
    width_ = width;
    height_ = height;
    depth_ = depth;
    prob_ = prob;
    startx_ = startx;
    starty_ = starty;
    startz_= startz;
    endx_ = endx;
    endy_ = endy;
    endz_=  endz;
    makeGrid();
    solveGrid();
    solveGridDFS();
    double actualDistance  = distance_[startx][starty][startz];
    double astarDistance = grid_[endx][endy][endz].f_;
    std::cout<<"We are printing the DFS shortest path versus the A*. If they are both the same, then we can conclude that A* has found the shortest path"<<std::endl;
    std::cout<<actualDistance<<' '<<astarDistance<<std::endl;
}
AStar::AStar(int width, int height, int depth)
{
    AStar(width, height, depth, 0.3, 0,0,0, width-1, height-1, depth-1);
}
void AStar::makeGrid()
{
    srand(time(0));
    Node* endNode = new Node(endx_, endy_, endz_, NULL);
    grid_ = new Node**[width_];
    for (int i = 0; i < width_; i++)
    {
        grid_[i] = new Node*[height_];
        for (int j = 0; j < height_; j++)
        {
            grid_[i][j] = new Node[depth_];
            for(int k = 0; k < depth_; k++)
            {
                double random = rand()%100;
                bool isObstacle = (prob_ * 100) > random;
                if(i == startx_ && j == starty_ && k == startz_)
                {
                    isObstacle = false;
                }
                Node* node =  new Node(i, j, k, endNode);
                node->setObstacle(isObstacle);
                grid_[i][j][k] = *node;
            }
        }
    }
    grid_[endx_][endy_][endz_] = *endNode;
}
void AStar::printGrid()
{
    for(int i = 0; i < width_; i++)
    {
        for(int j = 0; j < height_; j++)
        {
            for (int k = 0; k < depth_; k++)
            {
                std::cout<<grid_[i][j][k].str_<<' ';
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
        std::cout<<std::endl;
    }
}
void AStar::solveGrid()
{
    std::priority_queue<Node> q;
    grid_[startx_][starty_][startz_].g_ = 0;
    grid_[startx_][starty_][startz_].f_ = grid_[startx_][starty_][startz_].g_ + grid_[startx_][starty_][startz_].h_;

    q.push(grid_[starty_][starty_][startz_]);
    while(!q.empty())
    {
        std::cout<<"hello"<<' '<<q.top().y_<<q.top().z_<<std::endl;
        //Node now = q.top();
	Node *now = &grid_[q.top().x_][q.top().y_][q.top().z_];
        q.pop();
        now->isClosed_ = true;
        now->str_ = "2";
        std::cout << now->str_ << std::endl;
        if(now->x_ == endx_ && now->y_ == endy_ && now->z_ == endz_)
        {
            break;
        }
        int dy[26] = {1,1,1,0,0,-1,-1,-1, 1,1,1,0,0,-1,-1,-1,0, 1,1,1,0,0,-1,-1,-1,0};
        int dx[26] = {-1,0,1,-1,1,-1,0,1, -1,0,1,-1,1,-1,0,1,0, -1,0,1,-1,1,-1,0,1,0};
        int dz[26] = {0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, -1,-1,-1,-1,-1,-1,-1,-1,-1};
        for(int i = 0; i < 26; i++)
        {
            if(0 <= now->x_ + dx[i] && now->x_ + dx[i] < width_ && 0 <= now->y_ + dy[i] && now->y_ + dy[i] < height_ && 0 <= now->z_ + dz[i] && now->z_ + dz[i] < depth_)
            {
                Node *side = &grid_[now->x_ + dx[i]][now->y_ + dy[i]][now->z_ + dz[i]];
                if(side->isObstacle_) continue;
                if(side->isClosed_) continue;
                double distance = pow((pow((dx[i]), 2) + pow((dy[i]), 2) + pow((dz[i]), 2)), 0.5);
                if(now->g_ + distance < side->g_ || !side->isOpen_)
                {
                    side->g_ = now->g_ + distance;
                    side->f_ = side->g_ + side->h_;
                    side->parent_ = now;
                    if(!side->isOpen_)
                    {
                        side->isOpen_= true;
                        //q.push(&grid_[side->x_][side->y_][side->z_]);
			q.push(*side);
                    }
                }
            }
        }
    }
    makeParent(&grid_[endx_][endy_][endz_]);
    printGrid();
}
void AStar::makeParent(AStar::Node *node)
{
    grid_[node->x_][node->y_][node->z_].str_ = "3";
    if(node->parent_ == NULL) return;
    makeParent(node->parent_);
}
void AStar::solveGridDFS()
{
    distance_ = new double**[width_];
    for (int i = 0; i < width_; i++)
    {
        distance_[i] = new double*[height_];
        for (int j = 0; j < height_; j++)
        {
            distance_[i][j] = new double[depth_];
            for(int k = 0; k < depth_; k++)
            {
                distance_[i][j][k] = Node::infinity;
            }
        }
    }
    distance_[endx_][endy_][endz_] = 0;
    DFS(endx_, endy_, endz_);
    std::cout<<distance_[0][0][0];
}
void AStar::DFS(int x, int y, int z)
{
    int dy[26] = {1,1,1,0,0,-1,-1,-1, 1,1,1,0,0,-1,-1,-1,0, 1,1,1,0,0,-1,-1,-1,0};
    int dx[26] = {-1,0,1,-1,1,-1,0,1, -1,0,1,-1,1,-1,0,1,0, -1,0,1,-1,1,-1,0,1,0};
    int dz[26] = {0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, -1,-1,-1,-1,-1,-1,-1,-1,-1};
    for(int i = 0; i < 26; i++)
    {
        if(0 <= x + dx[i] && x + dx[i] < width_ && 0 <= y + dy[i] && y + dy[i] < height_ && 0 <= z + dz[i] && z + dz[i] < depth_)
        {
            Node side = grid_[x + dx[i]][y + dy[i]][z + dz[i]];
            if(side.isObstacle_) continue;
            float d = pow((pow((dx[i]), 2) + pow((dy[i]), 2) + pow((dz[i]), 2)), 0.5);
            if(distance_[side.x_][side.y_][side.z_] > distance_[x][y][z] + d)
            {
                distance_[side.x_][side.y_][side.z_] = distance_[x][y][z] + d;
                DFS(side.x_, side.y_, side.z_);
            }
        }
    }
}
AStar::Node::Node(int x, int y, int z, AStar::Node *end)
{
    x_ = x;
    y_ = y;
    z_ = z;
    if(end != NULL)
    {
        h_ = pow((pow((end->x_ - x_),2) + pow((end->y_ - y_),2) + pow((end->z_ - z_),2)),0.5);
    }
}
void AStar::Node::setObstacle(bool isObstacle)
{
    isObstacle_ = isObstacle;
    if (isObstacle_)
    {
        str_ = "1";
    } else
    {
        str_ = "0";
    }
}
AStar::Node::Node()
{
    Node(0,0,0,NULL);
}


}


int main()
{

    std::priority_queue<std::string> l;
    l.push("hello");
    l.push("bye");
    l.push("why");

    std::cout<<l.top()<<std::endl;

    std::cout<<l.top()<<std::endl;
    MuddSub::MotionPlanning::AStar* pStar = new   MuddSub::MotionPlanning::AStar(1,10,10);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
