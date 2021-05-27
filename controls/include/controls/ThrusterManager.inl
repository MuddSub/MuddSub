namespace MuddSub::Controls
{

template<size_t thrustDim, size_t numOfThrusters>
ThrusterManager<thrustDim, numOfThrusters>::ThrusterManager()
{
    resetConfiguration();
}

template<size_t thrustDim, size_t numOfThrusters>
typename ThrusterManager<thrustDim, numOfThrusters>::ThrustVector_t ThrusterManager<thrustDim, numOfThrusters>::calculateThrust(const ForceVector_t& f)
{
    ThrustVector_t thrustVec;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(tMatrix_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    size_t uDim = svd.matrixU().rows();
    size_t vDim = svd.matrixV().rows();
    Eigen::MatrixXd sMatrix = Eigen::MatrixXd::Zero(uDim, uDim);
    Eigen::MatrixXd sMatrixFull = Eigen::MatrixXd::Zero(vDim, uDim);

    for (size_t i = 0; i < svd.singularValues().rows(); ++i) 
    {
        sMatrix(i,i) = svd.singularValues()(i);
    }
    sMatrix = sMatrix.inverse();
    sMatrixFull.block(0,0,uDim, uDim) = sMatrix;
    thrustVec = svd.matrixV() * sMatrixFull * svd.matrixU().transpose() * f;

    return thrustVec;
}

// It's likely that this should be overloaded on any implementation other than the assumed default
// Thruster Manager dimensions
template<size_t thrustDim, size_t numOfThrusters>
void ThrusterManager<thrustDim, numOfThrusters>::resetConfiguration()
{
    std::string robotName;
        //Templated from VehicleDynamics.cc in /controls
        bool ready = false;
        while (!nh_.hasParam("dynamics_ready") && !ready) {
            ROS_WARN("Waiting for dynamics params to be loaded into server");
            ros::Duration(.1).sleep();
        }

        nh_.getParam("robot_name", thrustersInfo_.name_);
        std::string paramRoot = "/"+robotName+"/";
        nh_.getParam(paramRoot+"distance_from_center_of_gravity", thrustersInfo_.d_);
        nh_.getParam(paramRoot+"alpha", thrustersInfo_.alpha_);
        nh_.getParam(paramRoot+"phi", thrustersInfo_.phi_);

        thrustersInfo_.gamma_["lfr"] = thrustersInfo_.alpha_["lfr"] - thrustersInfo_.phi_["lfr"];
        thrustersInfo_.gamma_["lfl"] = thrustersInfo_.alpha_["lfl"] - thrustersInfo_.phi_["lfl"];
        thrustersInfo_.gamma_["lbr"] = thrustersInfo_.alpha_["lbr"] - thrustersInfo_.phi_["lbr"];
        thrustersInfo_.gamma_["lbl"] = thrustersInfo_.alpha_["lbl"] - thrustersInfo_.phi_["lbl"]; 
        
        // Fill tMatrix_ with config values
        // The rows correspond to x, y, z, yaw, pitch, roll (in that order)
        std::cout << thrustersInfo_.alpha_["lfr"] << std::endl;
        tMatrix_ << sin(thrustersInfo_.alpha_["lfr"]), sin(thrustersInfo_.alpha_["lfl"]), 
                    sin(thrustersInfo_.alpha_["lbr"]), sin(thrustersInfo_.alpha_["lbl"]), 0, 0, 0, 0,

                    cos(thrustersInfo_.alpha_["lfr"]), cos(thrustersInfo_.alpha_["lfl"]),
                    cos(thrustersInfo_.alpha_["lbr"]), cos(thrustersInfo_.alpha_["lbl"]), 0, 0, 0, 0,

                    0, 0, 0, 0, thrustersInfo_.d_["vfr"], thrustersInfo_.d_["vfl"], thrustersInfo_.d_["vbr"], 
                    thrustersInfo_.d_["vbl"],

                    thrustersInfo_.d_["lfr"] * sin(thrustersInfo_.gamma_["lfr"]),
                    thrustersInfo_.d_["lfl"] * sin(thrustersInfo_.gamma_["lfl"]),
                    thrustersInfo_.d_["lbr"] * sin(thrustersInfo_.gamma_["lbr"]),
                    thrustersInfo_.d_["lbl"] * sin(thrustersInfo_.gamma_["lbl"]), 0, 0, 0, 0,

                    0, 0, 0, 0, thrustersInfo_.d_["vfr"], thrustersInfo_.d_["vfl"], -thrustersInfo_.d_["vbr"], 
                    -thrustersInfo_.d_["vbl"],

                    0, 0, 0, 0, -thrustersInfo_.d_["vfr"], thrustersInfo_.d_["vfl"], -thrustersInfo_.d_["vbr"], 
                    thrustersInfo_.d_["vbl"];
                    
        std::cout << tMatrix_ << std::endl; 

}
}