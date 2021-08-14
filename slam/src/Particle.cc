#include "slam/Particle.hh"
#include "slam/util.hh"

namespace MuddSub::SLAM
{
unsigned int Particle::instances_ = 0;
std::random_device r;
std::default_random_engine Particle::randGenerator_ = std::default_random_engine{r()};

Particle::Particle(slamStateVector_t state, Parameters params):
  robotState_(state),
  params_(params)
{
  // if (params_ == NULL)
  // {
  //   Parameters p = {
  //     .n_ = 5,
  //     .velocitySigma_ = 0.04,
  //     .angleSigma_ = 0.0125,
  //     .slipSigma_ = 0.07,
  //     .rangeSigma_ = 0.075,
  //     .bearingSigma_ = 0.025,
  //     .rangeWeightStd_ = 0.03,
  //     .bearingWeightStd_ = 0.015
  //   };
  //   params_ = &p;
  // }
  //
  // id_ = Particle::instances_++;
  // std::cout<<"id "<<id_<<"\n";
  // 
  // ROS_INFO("Particle parameters: %d %f %f %f %f %f %f %f",
  //   params_.n_,
  //   params_.velocitySigma_,
  //   params_.angleSigma_,
  //   params_.slipSigma_,
  //   params_.rangeSigma_,
  //   params_.bearingSigma_,
  //   params_.rangeWeightStd_,
  //   params_.bearingWeightStd_
  // );

  n_ = params_.n_;
  velocityDistribution_ = std::normal_distribution<double>(0, params_.velocitySigma_);
  thetaDistribution_ = std::normal_distribution<double>(0, params_.angleSigma_);
  slipDistribution_ = std::normal_distribution<double>(0, params_.slipSigma_);
  weight_ = 1 / n_;

  // std::random_device r;
  // randGenerator_ = std::default_random_engine{r()};
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
  // std::random_device r;
  // randGenerator_ = std::default_random_engine{r()};

  velocity += velocityDistribution_(Particle::randGenerator_);
  thetaMeas += thetaDistribution_(Particle::randGenerator_);
  double slipVel = slipDistribution_(Particle::randGenerator_);

  // ROS_INFO("Theta: %f", thetaMeas);
  auto& theta = robotState_[THETA_IDX];

  theta = wrapToPi(thetaMeas);

  auto xVel = velocity*std::cos(theta);
  auto yVel = velocity*std::sin(theta);

  // ROS_INFO("Particle %p: XVel %f, YVel %f, slipVel %f, theta %f, dt %f, id %d", this, xVel, yVel, slipVel, theta, dt, id_);

  robotState_[X_IDX] += xVel*dt - slipVel*dt*std::sin(theta);
  robotState_[Y_IDX] += yVel*dt + slipVel*dt*std::cos(theta);
}

double Particle::correct(double time, unsigned int subject, double range, double bearing)
{
  if(subject < 5)
      throw std::runtime_error("Subject should not have been less than 5.");
  if(landmarkEKFs_.count(subject) == 0)
  {
    landmarkEKFs_[subject] = std::make_unique<EKF>(robotState_, range, bearing, params_);
    weight_ = 1/n_;
  }
  else
  {
    weight_ = landmarkEKFs_[subject]->correct(range, bearing, robotState_);
  }

  // ROS_INFO("Correcting particle: id %d, weight %f, n is %d", id_, weight_, n_);

  return weight_;
}

};
