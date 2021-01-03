#pragma once

#include <Eigen/Dense>
#include <unordered_map>

namespace MuddSub::SLAM
{
using slamStateVector_t = Eigen::Vector3d;
using slamControlVector_t = Eigen::Vector2d;
using slamMeasurementVector_t = Eigen::Vector2d;
using map_t = std::unordered_map<int, std::pair<double, double>>;

class KeyFrame
{
public:
  std::string type_;
  double time_;
  std::vector<double> data_;

  bool operator< (const KeyFrame& other) {
    return time_ < other.time_;
  };

};

inline bool operator< (const KeyFrame& frame, const double t){
  return frame.time_ < t;
};

struct State
{
  double x_{0}, y_{0}, theta_{0};
  map_t map_;
};

struct Parameters
{
	int numParticles;
	double velocitySigma, angleSigma, slipSigma, rangeSigma, bearingSigma, rangeWeightStd, bearingWeightStd;
};
}
