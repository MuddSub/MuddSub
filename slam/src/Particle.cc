#include "slam/Particle.hh"
#include "slam/util.hh"

namespace MuddSub::SLAM
{

Particle::Particle(unsigned long n, slamStateVector_t state):
                  n_(n),
                  slipDistribution_(0, slipSigma_),
                  velocityDistribution_(0, velocitySigma_),
                  thetaDistribution_(0, angleSigma_),
                  robotState_(state),
                  weight_(1/n)
{
  robotState_[THETA_IDX] = wrapToPi(robotState_[THETA_IDX]);
}

Particle::Particle(const Particle& other):
                   n_(other.n_),
                   slipDistribution_(other.slipDistribution_),
                   velocityDistribution_(other.velocityDistribution_),
                   thetaDistribution_(other.thetaDistribution_),
                   robotState_(other.robotState_)
{
  for(const auto& entry : other.landmarkEKFs_)
  {
    landmarkEKFs_[entry.first] = {std::make_unique<EKF>(*entry.second)};
  }
}

void Particle::propagateMotion(double velocity, double thetaMeas, double dt)
{
  velocity += velocityDistribution_(randGenerator_);
  thetaMeas += thetaDistribution_(randGenerator_);
  double slipVel = slipDistribution_(randGenerator_);


  // ROS_INFO("Theta: %f", thetaMeas);
  auto& theta = robotState_[THETA_IDX];

  theta = wrapToPi(thetaMeas);

  auto xVel = velocity*std::cos(theta);
  auto yVel = velocity*std::sin(theta);
  // ROS_INFO("XVel %f, YVel %f, theta %f, dt %f", xVel, yVel, theta, dt);

  robotState_[X_IDX] += xVel*dt - slipVel*dt*std::sin(theta);
  robotState_[Y_IDX] += yVel*dt + slipVel*dt*std::cos(theta);
}

double Particle::correct(double time, unsigned int subject, double range, double bearing)
{
  if(subject < 5)
      throw std::runtime_error("Subject should not have been less than 5.");
  if(landmarkEKFs_.count(subject) == 0)
  {
    landmarkEKFs_[subject] = std::make_unique<EKF>(robotState_, range, bearing);
    weight_ = 1/n_;
  }
  else
  {
    weight_ = landmarkEKFs_[subject]->correct(range, bearing, robotState_);
  }

  return weight_;
}

};
