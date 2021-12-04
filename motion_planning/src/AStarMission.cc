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
            pastObstacles_(),
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

    std::vector<double> AStarMission::getPoseVector(geometry_msgs::PoseStamped poseStamped)
    {
        std::vector<double> v;
        
        std::vector<double> q;
        q.push_back(poseStamped.pose.orientation.x);
        q.push_back(poseStamped.pose.orientation.y);
        q.push_back(poseStamped.pose.orientation.z);
        q.push_back(poseStamped.pose.orientation.w);

        std::vector<double> e = MuddSub::MotionPlanning::AStarMission::quaternionToEuler(q);
        v.push_back(poseStamped.pose.position.x);
        v.push_back(poseStamped.pose.position.y);
        v.push_back(poseStamped.pose.position.z);

        v.push_back(e[0]);
        v.push_back(e[1]);
        v.push_back(e[2]);

        v.push_back(poseStamped.header.stamp.toSec());

        return v; 
    }

    geometry_msgs::PoseStamped AStarMission::getPoseStamped(std::vector<double> v)
    {
        geometry_msgs::PoseStamped poseStamped;
        ros::Time time(v[6]);
        poseStamped.header.stamp = time;
        poseStamped.pose.position.x = v[0];
        poseStamped.pose.position.y = v[1];
        poseStamped.pose.position.z = v[2];

        std::vector<double> e;
        e.push_back(v[3]);
        e.push_back(v[4]);
        e.push_back(v[5]);
        std::vector<double> q = MuddSub::MotionPlanning::AStarMission::eulerToQuaternion(e);

        poseStamped.header.stamp = time;
        poseStamped.pose.orientation.x = q[0];
        poseStamped.pose.orientation.y = q[1];
        poseStamped.pose.orientation.z = q[2];
        poseStamped.pose.orientation.w = q[3];
	return poseStamped;

    }

    geometry_msgs::Point AStarMission::getPoint(std::vector<double> v)
    {
        geometry_msgs::Point p;
        p.x = v[0];
        p.y = v[1];
        p.z = v[2];
        return p;
    }
    std::vector<double> AStarMission::getPointVector(geometry_msgs::Point point)
    {
        std::vector<double> v;
        v.push_back(point.x);
        v.push_back(point.y);
        v.push_back(point.z);
        return v;
    }

    std::vector<std::vector<double>> AStarMission::convertPoints(std::vector<geometry_msgs::Point> points)
    {
        std::vector<std::vector<double>> converted;
        
        for(int i = 0; i < points.size(); i++)
        {
            converted.push_back(AStarMission::getPointVector(points[i]));
        }

        return converted;
    }

    std::vector<double> AStarMission::eulerToQuaternion (std::vector<double> e)
    {
        std::vector<double> q;
        double yaw = e[0];
        double pitch = e[1];
        double roll = e[2];
        
        double qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        double qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        double qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
        double qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
        
        q.push_back(qx);
        q.push_back(qy);
        q.push_back(qz);
        q.push_back(qw);
        
        
        return q;

    }
    
    
    
    std::vector<double> AStarMission::quaternionToEuler (std::vector<double> q)
    {
        double x = q[0];
        double y = q[1];
        double z = q[2];
        double w = q[3];

        std::vector<double> e;

        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + y * y);
        double roll = atan2(t0, t1);
        double t2 = +2.0 * (w * y - z * x);
        if(t2 > +1.0)
        {
            t2 = +1.0;
        }

        if(t2 < -1.0)
        {
            t2 = -1.0;
        }
        double pitch = asin(t2);
        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (y * y + z * z);
        double yaw = atan2(t3, t4);
        
        e.push_back(yaw);
        e.push_back(pitch);
        e.push_back(roll);

        return e;
    }

    void AStarMission::addMotion(std::vector<double> pose)
    {
        if(goals_[0][2] == "None")
        {
            current_->addMotion(AStar::None, pose, "");
        }else if(goals_[0][2] == "SineTraversalMovement")
        {
            current_->addMotion(AStar::sinTraversalPath, pose, goals_[0][2]);
        }else if(goals_[0][2] == "RotationalMovement" )
        {
            current_->addMotion(AStar::rotationalMovement, pose, goals_[0][4]);
        }
    }

    void AStarMission::printPath()
    {
        std::cout<<"Printing Path"<<std::endl;
        for(int i = 0; i < path_.size(); i++)
        {
            for(int j = 0; j < path_[i].size(); j++)
            {
                std::cout<<path_[i][j]<<' ';
            }
            std::cout<<std::endl;
        }
    }

    void AStarMission::convertPath()
    {
        for(int i = 0; i < path_.size(); i++)
        {
            finalPath_.push_back(MuddSub::MotionPlanning::AStarMission::getPoseStamped(path_[i]));
        }
    }


    void AStarMission::recurse(geometry_msgs::PoseStamped poseStamped, double time)
    {
        std::vector<double> pose = MuddSub::MotionPlanning::AStarMission::getPoseVector(poseStamped);       
        //double time = pose[6];
        //If an AStar object is created with the specific goal
        std::cout<<"recurse, time: "<<time<<std::endl;
        if(current_)
        {
            if(pastObstacles_ != obstacles_)
            {
                pastObstacles_ = obstacles_;
                current_->addObstacles(convertPoints(obstacles_));
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
            addMotion(pose);
            //current_->addTime();
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
            current_ = new AStar(convertPoints(obstacles_), width_, height_, depth_, pose[0], pose[1], pose[2], goal[0], goal[1], goal[2], "", time, velocity_);
            current_->printGrid();
            pastObstacles_ = obstacles_;
            current_->solveGrid();
            current_->printGrid();
            addMotion(pose);
            std::cout<<"Added motion"<<std::endl;
            //current_->addTime();
            path_ = current_->path_;
        }

        convertPath();
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
