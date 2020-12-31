#pragma once

#include <vector>
#include <assert.h>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <array>
#include <utility>

#include "matplotlib-cpp/matplotlibcpp.h"
#include "cpptqdm/tqdm.h"
#include "slam/Particle.hh"
#include "slam/EKF.hh"
#include "slam/DataLoader.hh"
#include "slam/util.hh"
#include "slam/Types.hh"

namespace MuddSub::SLAM
{

class FastSLAM
{

public:

  FastSLAM(const int& datasetId, const int& robotId);

  void createParticles();

  void runFastSLAM();

  State getStateMaxWeight();

  State getStateAvg();

  inline double rms(std::vector<double> truth, std::vector<double> estimate)
  {
    assert(truth.size() == estimate.size());
    std::vector<double> errors;

    for(int i = 0; i < truth.size(); ++i)
      errors.push_back(truth[i] - estimate[i]);

    return std::sqrt(std::accumulate(errors.begin(), errors.end(), 0));
  };

  inline double euclid(double x, double xt, double y, double yt)
  {
    return std::sqrt(std::pow(x-xt, 2) + std::pow(y-yt, 2));
  }

  double euclidRMS(std::vector<State> stateTruths, std::vector<State> stateEstimates);


private:

  static const unsigned int n_{150};
  static const unsigned int numSteps_{15000};
  const unsigned int robotID_{1};
  const unsigned int datasetID_{1};
  static const unsigned int estimateSnapshotInterval_{100};
  int snapshotCounter_{0};

  std::array<Particle, n_> particles_;

  std::vector<State> stateLogs_;
  std::vector<double> timeSeries_;

  std::vector<std::array<Particle, n_>> particleLogs_;


  DataLoader data_;

  std::array<Particle, n_> resample(const std::array<Particle, n_>& input,
                                    const std::array<double, n_>& weights,
                                    bool normalized);

};


}
