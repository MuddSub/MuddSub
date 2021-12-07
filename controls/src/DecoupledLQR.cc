#include "controls/DecoupledLQR.hh"


namespace MuddSub::Controls
{

//Compute the error
stateVector_t DecoupledLQR::computeError(const stateVector_t& state,
                                         const stateVector_t& setpoint) const
{


  // translational position
  Eigen::Matrix<double, 1, 3> position, attitude;

  // The error in position is just difference of front of state and setpoint vectors
  position = state.segment<3>(0) - setpoint_.segment<3>(0);

  // Similar for linear velocity and angular velocities
  Eigen::Matrix<double, 1, 6> velocity;
  velocity = state.segment<6>(6) - setpoint_.segment<6>(6);

  // For angular position (verbose for clarity...)
  auto angleErr = [](const double& stateElement, const double& setpointElement)
  {
    double stateNorm = fmod(stateElement, 2*M_PI);
    if(stateNorm < 0) stateNorm += 2*M_PI;

    double setpointNorm = fmod(setpointElement, 2*M_PI);
    if(setpointNorm < 0) setpointNorm += 2*M_PI;

    double result = fmod(stateNorm - setpointNorm, 2*M_PI);
    if(result > M_PI)
      result -= 2*M_PI;
    else if(result < -M_PI)
      result += 2*M_PI;

    return result;
  };

  for(int i = 0; i < 3; ++i)
    attitude[i] = angleErr(state[i+3], setpoint[i+3]);

  // pack 'em up and send 'em off
  Eigen::Matrix<double, 1, stateDim> result;
  result << position, attitude, velocity;
  return result;
}

// Compute control action
void DecoupledLQR::computeControl(const stateVector_t& state,
                                  const double& t,
                                  controlVector_t& controlAction)
{
  // Update the PID controllers
  double deltaT = t - previousTime_;
  previousTime_ = t;

  ct::core::SystemLinearizer<stateDim,controlDim, double> linearizer(vehicleDynamics_);

  // Our system linearization doesn't depend on u, so we can just make it zeros
  Eigen::Matrix<double, controlDim, 1> u = Eigen::Matrix<double, controlDim, 1>::Zero();

  Eigen::Matrix<double, stateDim, stateDim> A = linearizer.getDerivativeState(state,u,t);
  Eigen::Matrix<double, stateDim, controlDim> B = linearizer.getDerivativeControl(state,u,t);

  auto err = computeError(state, setpoint_);
  std::cout << "Error: " << err.format(eigenInLine) << std::endl;

  // Now that we've linearized about the entire state (which needs roll/pitch info)
  //     we actually only keep x,y,z,yaw and those velocities
  const auto& partitionedA = partitionAMatrix(A);
  const auto& partitionedB = partitionBMatrix(B);


  Eigen::Matrix<double, stateDimLQR, 1> error8DoF;
  error8DoF << err.segment<3>(0), err[5], err.segment<3>(6), err[11];

  // TODO: This is a very expensive way to do it... shouldn't poll on every iteration
  double kQ, kR;
  if(nh_.getParam("/controls/kQ", kQ)) lqr8DoF_.setQUniformDiag(kQ);
  if(nh_.getParam("/controls/kR", kR)) lqr8DoF_.setQUniformDiag(kR);

  const auto& controlActionLQR = lqr8DoF_.computeControl(partitionedA, partitionedB, error8DoF);

  double controlActionRoll = rollPid_.update(err[3], deltaT);
  double controlActionPitch = pitchPid_.update(err[4], deltaT);

  controlAction << controlActionLQR.segment<3>(0), controlActionRoll,
                   controlActionPitch, controlActionLQR[3];

  std::cout << "Control Action Computed " << controlAction.format(eigenInLine) << std::endl;
}

// Throw out roll and pitch
Eigen::Matrix<double, DecoupledLQR::stateDimLQR, DecoupledLQR::stateDimLQR>
  DecoupledLQR::partitionAMatrix(const Eigen::Matrix<double, stateDim, stateDim>& A)
{
  // This all uses eigen block operations: matrix.block(i,j,p,q) gives the block of
  //  size (p,q) starting at (i,j)


  // Plan of attack: From right to left, cut out the unwanted columns by copying the good ones
  // over in blocks. Then, do the same from bottom to top.
  Eigen::MatrixXd result = A;

  //reminder: state is [x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw']

  //pull the last column to the 9th, overwriting cols with roll' with yaw'
  result.block(0, 9, 12, 1) = result.block(0, 11, 12, 1);

  // Now the matrix is (horizontally) [x,y,z,roll, pitch, yaw, x', y', z',yaw']

  result.block(0, 3, 12, 5) = result.block(0, 5, 12, 5);

  // Now, it's [x,y,z,yaw,x',y',z',yaw'] (as intended). Repeat vertically:

  result.block(9, 0, 1, 8) = result.block(11, 0, 1, 8);
  result.block(3, 0, 5, 8) = result.block(5, 0, 5, 8);

  // Fix the size
  result.conservativeResize(8,8);
  return static_cast<Eigen::Matrix<double, 8, 8>>(result);
}

//This is very similar to partitioning the A matrix
Eigen::Matrix<double, DecoupledLQR::stateDimLQR, DecoupledLQR::controlDimLQR>
  DecoupledLQR::partitionBMatrix(const Eigen::Matrix<double, stateDim, controlDim>& B)
{
  Eigen::MatrixXd result = B;

  // Row partitioning is exact same as above (but we start with only 6 columns)
  result.block(9, 0, 1, 6) = result.block(11, 0, 1, 6);
  result.block(3, 0, 5, 6) = result.block(5, 0, 5, 6);

  // Columns we only have to cut out pitch and roll from the wrench vector
  result.block(0, 3, 8, 1) = result.block(0, 5, 8, 1);
  result.conservativeResize(8,4);
  return static_cast<Eigen::Matrix<double, 8, 4>>(result);
}
}
