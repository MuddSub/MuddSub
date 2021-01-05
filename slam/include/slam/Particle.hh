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
  Particle(slamStateVector_t state, Parameters params = Parameters());

  Particle(const Particle& rhs);

  unsigned int id_{0};

private:

  void propagateMotion(double velocity, double thetaMeas, double dt);

  double correct(double time, unsigned int subject, double range, double bearing);

  std::unordered_map<int, std::shared_ptr<EKF>> landmarkEKFs_;

  double weight_{1};
  Parameters params_;
  unsigned int n_;

  static std::default_random_engine randGenerator_;

  std::normal_distribution<double> slipDistribution_, thetaDistribution_,
                                   velocityDistribution_;

  slamStateVector_t robotState_{0, 0, 0};

  static unsigned int instances_;

  static const unsigned short X_IDX{0};
  static const unsigned short Y_IDX{1};
  static const unsigned short THETA_IDX{2};

  friend class FastSLAM;

  // map_t map;

};

}
