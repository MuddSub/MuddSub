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

  FastSLAM(const int& datasetId, const int& robotId, Parameters* params = NULL);

  void createParticles();

  double runFastSLAM();

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

  // const Parameters params;
  //question
  static const unsigned int numSteps_{15000};
  const unsigned int robotID_{1};
  const unsigned int datasetID_{1};
  static const unsigned int estimateSnapshotInterval_{100};
  int snapshotCounter_{0};
  Parameters* params_;
  unsigned int n_;

  std::vector<Particle> particles_;

  std::vector<State> stateLogs_;
  std::vector<double> timeSeries_;

  std::vector<std::vector<Particle>> particleLogs_;


  DataLoader data_;

  std::vector<Particle> resample(const std::vector<Particle>& input,
                                    const std::vector<double>& weights,
                                    bool normalized);

};


}
