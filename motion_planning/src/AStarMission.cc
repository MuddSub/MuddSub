#include "motion_planning/AStarMission.hh"
namespace MuddSub::MotionPlanning
{
    /*
     * each vector in goals is of size three, with "goal", "linear movement", and "rotational movement", data needed
     * ex: ["1 1 1 0 pi/2 pi", "goToLocation", "None", ""]
     * ex: ["buoy_fairy", "goToTarget", "SineTraversalMovement", "10 10 0"]
     */
    AStarMission::AStarMission(std::vector<std::vector<std::string>> &goals, double velocity, int width, int height, int depth):
            goals_(goals),
            pastObstacles_(NULL),
            velocity_(velocity),
            current_(nullptr),
            width_(width),
            height_(height),
            depth_(depth),
            targets_(),
            path_(),
            obstacles_()
    {

    }

    void AStarMission::addGoals(std::vector<std::vector<std::string>> goals)
    {
        goals_.insert(std::end(goals_), std::begin(goals), std::end(goals));
    }

    void AStarMission::isSucessful(bool success)
    {
        if(success)
        {
            goals_.erase(goals_.begin(), goals_.begin()+1);
            current_ = nullptr;
        }
    }


    std::vector<double> AStarMission::getPose(std::string poseString)
    {
        //check if it is in the map and return the vector
        std::vector<double> pose;
        int index = 0;
        while(true)
        {
            size_t foundString = poseString.find(' ', index);
            if(foundString == std::string::npos)
            {
                if(index <= poseString.size())
                {
                    std::string substring = poseString.substr(index, poseString.size());
                    int elem = std::stoi(substring);
                    pose.push_back(elem);
                }
                break;
            }

            std::string substring = poseString.substr(index, foundString); //TODO: should this be: foundString- index
            double elem = std::stod(substring);
            pose.push_back(elem);
            index = foundString + 1;
        }
        return pose;
    }

    void AStarMission::addMotion()
    {
        if(goals_[0][2] == "None")
        {
            current_->addMotion(AStar::None, "");
        }else if(goals_[0][2] == "SineTraversalMovement")
        {
            current_->addMotion(AStar::sinTraversalPath, goals_[0][3]);
        }else if(goals_[0][2] == "RotationalMovement" )
        {
            current_->addMotion(AStar::rotationalMovement, goals_[0][3]);
        }
    }

    void AStarMission::printPath()
    {
        for(int i = 0; i < path_.size(); i++)
        {
            for(int j = 0; j < path_[i].size(); j++)
            {
                std::cout<<path_[i][j]<<' ';
            }
            std::cout<<std::endl;
        }
    }


    void AStarMission::recurse(std::vector<double> pose, double time)
    {
        //If an AStar object is created with the specific goal
        std::cout<<"recurse, time: "<<time<<std::endl;
        if(current_)
        {
            if(pastObstacles_ != obstacles_)
            {
                pastObstacles_ = obstacles_;
                current_->addObstacles(obstacles_);
                current_->printGrid();
                current_->updatePose(pose[0], pose[1], pose[2]);
                current_->solveGrid();
                current_->printGrid();
            }else
            {
                current_->updatePose(pose[0], pose[1], pose[2]);
                current_->resolveGrid();
                current_->printGrid();
            }
            addMotion();
            //current_->addTime(1);
            path_ = current_->path_;
        }else
        {
            if(goals_.size() == 0)
            {
                std::cout<<"No Mission"<<std::endl;
                return;
            }
            std::string goalString = goals_[0][0];
            std::vector<double> goal = getPose(goalString);
            current_ = new AStar(obstacles_, width_, height_, depth_, pose[0], pose[1], pose[2], goal[0], goal[1], goal[2], "", time, velocity_);
            current_->printGrid();
            pastObstacles_ = obstacles_;
            current_->solveGrid();
            current_->printGrid();
            addMotion();
            std::cout<<"Added motion"<<std::endl;
            //current_->addTime(1);
            path_ = current_->path_;
        }
    }

    AStarMission::AStarMission() :
            goals_(),
            targets_(),
            path_(),
            obstacles_(),
            current_(nullptr),
            width_(0)
    {

    }
}