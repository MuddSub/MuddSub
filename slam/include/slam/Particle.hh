#pragma once

#include <cmath>
#include <unordered_map>
#include <stdexcept>
#include <memory>
#include <random>

#include "slam/EKF.hh"
#include "slam/Types.hh"

namespace MuddSub::SLAM
{

class Particle
{

public:
  Particle() = default;
  Particle(unsigned long n, slamStateVector_t state);

  Particle(const Particle& rhs);


private:

  void propagateMotion(double velocity, double thetaMeas, double dt);

  double correct(double time, unsigned int subject, double range, double bearing);

  std::unordered_map<int, std::shared_ptr<EKF>> landmarkEKFs_;

  double weight_{1};

  double velocitySigma_{0.07};
  double angleSigma_{0.025};
  double slipSigma_{0.125};

  unsigned long n_;
  
  std::default_random_engine randGenerator_;

  std::normal_distribution<double> slipDistribution_, thetaDistribution_,
                                   velocityDistribution_;

  slamStateVector_t robotState_{0,0,0};


  static const unsigned short X_IDX{0};
  static const unsigned short Y_IDX{1};
  static const unsigned short THETA_IDX{2};

  friend class FastSLAM;

  // map_t map;

};

}
