#pragma once

#include <Eigen/Dense>
#include "slam/util.hh"
#include "slam/Types.hh"

#include <vector>
#include <cmath>
#include <memory>

namespace MuddSub::SLAM
{
using slamStateVector_t = Eigen::Vector3d;
using slamControlVector_t = Eigen::Vector2d;
using slamMeasurementVector_t = Eigen::Vector2d;

class EKF
{
private:
    static const int n_{2};
    Eigen::Matrix2d sigmaZ_;
    Eigen::Vector2d stateEstimate_;
    Eigen::Matrix<double, n_, n_> stateCovariance_;
    std::vector<Eigen::Matrix<double, 2, 1>> stateEstimateLogs_;
    std::vector<Eigen::Matrix<double, n_, n_>> stateCovarianceLogs_;


    friend class FastSLAM;
public:

  EKF(slamStateVector_t robotState, double range, double bearing);

  void setMeasurementCovariance(Eigen::Matrix2d cov);

  slamMeasurementVector_t measurementModel(slamStateVector_t robotState);
  Eigen::Matrix2d computeMeasurementJacobian(double range, double bearing, slamStateVector_t robotState);

  // Returns the probability of getting the measurement
  double correct(double range, double bearing, slamStateVector_t robotState);

};
}
