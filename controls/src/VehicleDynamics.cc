#include "controls/VehicleDynamics.hh"

#include <iostream>
#include <math.h>
#include <chrono>

namespace MuddSub::Controls
{


// Nonlinear vehicle dynamics
VehicleDynamics::VehicleDynamics()
{

  // Wait for parameters to be loaded onto server (read from URDF)
  bool ready = false;
  while(!nh_.hasParam("dynamics_ready") && !ready)
  {
    ROS_WARN("Waiting for dynamics params to be loaded into server");
    ros::Duration(.1).sleep();
  }

  // Load parameters for vehicle dynamics
  nh_.getParam("robot_name", robotInfo_.name_);
  std::string paramRoot = "dynamics/"+robotInfo_.name_+"/";
  nh_.getParam(paramRoot+"center_of_gravity", robotInfo_.centerOfGravity_);
  nh_.getParam(paramRoot+"center_of_buoyancy", robotInfo_.centerOfBuoyancy_);
  nh_.getParam(paramRoot+"mass", robotInfo_.mass_);
  nh_.getParam(paramRoot+"buoyancy", robotInfo_.buoyancy_);
  nh_.getParam(paramRoot+"added_mass", robotInfo_.addedMass_);
  nh_.getParam(paramRoot+"linear_damping_coeffs", robotInfo_.linearDampingCoefficients_);
  nh_.getParam(paramRoot+"rate", rate_);
  nh_.getParam(paramRoot+"inertia", robotInfo_.inertia_);

  // Construct the inertia tensor, represented as 3x3 matrix (Eigen3d)
  robotInfo_.inertiaTensor_ <<
    robotInfo_.inertia_["ixx"], -1*robotInfo_.inertia_["ixy"], -1*robotInfo_.inertia_["ixz"],
    -1*robotInfo_.inertia_["ixy"], robotInfo_.inertia_["iyy"], -1*robotInfo_.inertia_["iyz"],
    -1*robotInfo_.inertia_["ixz"], -1*robotInfo_.inertia_["iyz"], robotInfo_.inertia_["izz"];

  ROS_INFO("Robot dynamics loaded into VehicleDynamics");

  robotInfo_.centerOfGravityVector_ << robotInfo_.centerOfGravity_["x"],
                                       robotInfo_.centerOfGravity_["y"],
                                       robotInfo_.centerOfGravity_["z"];

  robotInfo_.centerOfBuoyancyVector_ << robotInfo_.centerOfBuoyancy_["x"],
                                        robotInfo_.centerOfBuoyancy_["y"],
                                        robotInfo_.centerOfBuoyancy_["z"];

}

// Clone required by Control Toolbox (pure virtual) replaced with smart pointers
VehicleDynamics* VehicleDynamics::clone() const
{
  return new VehicleDynamics(*this);
}

// Clone the current instance
std::shared_ptr<VehicleDynamics> VehicleDynamics::smartClone() const
{
  return std::shared_ptr<VehicleDynamics>(clone());
}


// Given the current state and control input, find the derivative.
// Note: Since this function is called as part of the numerical differentiation
//       routine, speed is critical.
void VehicleDynamics::computeControlledDynamics(const stateVector_t &x,
                                                const ctTime_t& t,
                                                const controlVector_t &u,
                                                stateVector_t &derivative)
{
  // Segment the state vector into position, velocity, attitute, and rotational velocity.
  // Names kept brief for the sake of large equations.
  const auto& position = x.segment<3>(0);
  const auto& attitude = x.segment<3>(3);
  const auto& v1 = x.segment<3>(6);
  const auto& v2 = x.segment<3>(9);

  // Centripital and coriolis forces
  const auto& C = buildCMatrix(v1, v2);

  // Compute the force of gravity, in the frame of the robot
  const auto& Gf = buildGravityMatrix(attitude);

  // Find transformation between world frame and robot frame
  const auto& Jn = buildJnMatrix(attitude);

  // The generalized mass matrix comprises actual mass and hydrodynamic added mass
  // Todo: This doesn't need to be calculated at every iteration
  const auto& M = buildMassMatrix();
  Eigen::Matrix<double, 6, 6> mInv;

  // TODO: Make this faster!
  if(M.determinant() == 0)
    mInv = Eigen::Matrix<double, 6, 6>::Zero();
  else
    mInv = M.inverse();

  // In state space form: x' = f(x) + g(u)
  // Refer to Chin 2013, p. 138 for a full explanation.
  Eigen::Matrix<double, 12, 12> f1;
  f1 << Eigen::Matrix<double, 6, 6>::Zero(), Jn,
        Eigen::Matrix<double, 6, 6>::Zero(), mInv;

  Eigen::Matrix<double, 12, 1> f2, f, g, xT;
  xT = x.transpose();

  Eigen::Matrix<double, 6, 1> generalizedGravity = -1*(mInv*Gf);
  f2 << Eigen::Matrix<double, 6, 1>::Zero(), generalizedGravity;

  f = f1*xT + f2;
  Eigen::Matrix<double, 6, 1> controlForces = mInv*u;
  g << Eigen::Matrix<double, 6, 1>::Zero(), controlForces;
  derivative = (f+g).transpose();
}

void VehicleDynamics::computeDynamics(const stateVector_t& state, const ctTime_t& t, stateVector_t& deriv)
{
  const auto& controller = getController();
  controlVector_t controlAction;
  if(controllerSet_)
  {
    controller->computeControl(state, t, controlAction);
  }
  else
    controlAction = controlVector_t::Zero();
  computeControlledDynamics(state, t, controlAction, deriv);
}

// Coriolis and centripital forces
Eigen::Matrix<double, 6, 6> VehicleDynamics::buildCMatrix(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  Eigen::Matrix3d C12 = -1*robotInfo_.mass_*
                              (
                                S(v1) + S(v2)*S(robotInfo_.centerOfGravityVector_)
                              );

  Eigen::Matrix3d C21 = -1*C12.transpose();

  Eigen::Matrix3d C22 = -1 * S(robotInfo_.inertiaTensor_* v2);

  // Fossen 2011, p. 65
  Eigen::Matrix<double, 6, 6> Crb;
  Crb << Eigen::Matrix3d::Zero(), C12,
         C21, C22;
  Eigen::Matrix<double, 6, 6> Ca;

  // Fossen 2011, p. 121
  Ca << 0, 0, 0, 0, -1*robotInfo_.addedMass_["z"]*v1(2), robotInfo_.addedMass_["y"]*v1(1),
        0, 0, 0, robotInfo_.addedMass_["z"]*v1(2), 0, -1*robotInfo_.addedMass_["x"]*v1(0),
        0, 0, 0, -1*robotInfo_.addedMass_["y"]*v1(1), robotInfo_.addedMass_["x"]*v1(0), 0,
        0, -1*robotInfo_.addedMass_["z"]*v1(2), robotInfo_.addedMass_["y"]*v1(1), 0, -1*robotInfo_.addedMass_["r"]*v2(2), robotInfo_.addedMass_["q"]*v2(1),
        robotInfo_.addedMass_["z"]*v1(2), 0, -1*robotInfo_.addedMass_["x"]*v1(0), robotInfo_.addedMass_["r"]*v2(2), 0, -1*robotInfo_.addedMass_["p"]*v2(0),
        -1*robotInfo_.addedMass_["y"]*v1(1), robotInfo_.addedMass_["x"]*v1(0), 0, -1*robotInfo_.addedMass_["q"]*v2(1), robotInfo_.addedMass_["p"] * v2(0), 0;

  return Ca + Crb;

}

// Generalized mass matrix
Eigen::Matrix<double, 6, 6> VehicleDynamics::buildMassMatrix()
{
  Eigen::Matrix<double, 6, 6> Mrb, Ma;
  Eigen::Matrix3d Mm = I_*robotInfo_.mass_;

  Mrb << Mm, -1*robotInfo_.mass_*S(robotInfo_.centerOfGravityVector_),
         robotInfo_.mass_*S(robotInfo_.centerOfGravityVector_), robotInfo_.inertiaTensor_;

  Eigen::Matrix<double, 1, 6> addedMassVector;
  std::map<std::string, double>& addMass = robotInfo_.addedMass_;
  addedMassVector << addMass["x"], addMass["y"], addMass["z"], addMass["p"], addMass["q"], addMass["r"];
  Ma = addedMassVector.asDiagonal();
  return Ma + Mrb;
}

// skew-symmetric matrix for 3-vector cross products
Eigen::Matrix3d VehicleDynamics::S(const Eigen::Vector3d& vec) const
{
  Eigen::Matrix3d S;
  S << 0, -1*vec(2), vec(1),
       vec(2), 0, -1*vec(0),
       -1*vec(1), vec(0), 0;
  return S;
}

// Gravity in robot's frame
Eigen::Matrix<double, 6, 1> VehicleDynamics::buildGravityMatrix(const Eigen::Vector3d& attitude)
{

  // Here we re-name variables for the sake of making equations semi-readable, and in the
  // notation of Fossen, 2011, p. 60

  double cPhi, sPhi, cTheta, sTheta, tTheta, cTsi, sTsi;
  cPhi = cos(attitude(0));
  sPhi = sin(attitude(0));
  cTheta = cos(attitude(1));
  sTheta = sin(attitude(1));
  tTheta = tan(attitude(1));
  cTsi = cos(attitude(2));
  sTsi = sin(attitude(2));

  // Weight and buoyancy.
  const auto& w = gravity_*robotInfo_.mass_;
  const auto& b = gravity_*robotInfo_.buoyancy_;

  // effective weight
  const auto& ew = w-b;

  // Centers of gravity and buoyancy.
  const auto& cgX = robotInfo_.centerOfGravity_["x"];
  const auto& cgY = robotInfo_.centerOfGravity_["y"];
  const auto& cgZ = robotInfo_.centerOfGravity_["z"];
  const auto& cbX = robotInfo_.centerOfBuoyancy_["x"];
  const auto& cbY = robotInfo_.centerOfBuoyancy_["y"];
  const auto& cbZ = robotInfo_.centerOfBuoyancy_["z"];

  // Fossen 2013, p. 60
  Eigen::Matrix<double, 6, 1> result;
  result << ew*sTheta,
           -1*ew*cTheta*sPhi,
           -1*ew*cTheta*cPhi,
           -1*(cgY*w-cbY*b)*cTheta*cPhi+(cgZ*w-cbZ*b)*cTheta*sPhi,
           (cgZ*w-cbZ*b)*sTheta+(cgX*w-cbX*b)*cTheta*cPhi,
           -1*(cgX*w-cbX*b)*cTheta*sPhi-(cgY*w-cbY*b)*sTheta;

  return result;
}

Eigen::Matrix<double, 6, 6> VehicleDynamics::buildJnMatrix(const Eigen::Vector3d& attitude)
{
  // Todo: this shouldn't be copy-pasta'd
  double cPhi, sPhi, cTheta, sTheta, tTheta, cTsi, sTsi;
  cPhi = cos(attitude(0));
  sPhi = sin(attitude(0));
  cTheta = cos(attitude(1));
  sTheta = sin(attitude(1));
  tTheta = tan(attitude(1));
  cTsi = cos(attitude(2));
  sTsi = sin(attitude(2));

  Eigen::Matrix3d Rb, Tt;

  // Fossen 2011, p. 22
  Rb  << cTsi*cTheta, (-sTsi*cPhi + cTsi*sTheta*sPhi), (sTsi*sPhi+cTsi*cPhi*sTheta),
          sTsi*cTheta, (cTsi*cPhi + sPhi*sTheta*sTsi), (-cTsi*sPhi+sTheta*sTsi*cTheta),
          -sTheta, cTheta*sPhi, cTheta*cPhi;

  // Fossen 2011, p. 25
  Tt << 1, sPhi*tTheta, cPhi*tTheta,
        0, cPhi, -sPhi,
        0, (sPhi/cTheta), (cPhi/cTheta);

  Eigen::Matrix<double, 6, 6> Jn;
  // Fossen 2011, p. 26
  Jn << Rb, Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), Tt;

  return Jn;
}

void VehicleDynamics::setController(const controllerPtr_t& controller)
{
  controllerSet_ = true;
  controller_ = controller;
}

} // Namespace MuddSub::Controls
