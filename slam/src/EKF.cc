#include <Eigen/Dense>

#include "slam/EKF.hh"

namespace MuddSub::SLAM
{

EKF::EKF(slamStateVector_t robotState, double range, double bearing, Parameters params):
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

    // ROS_INFO("EKF parameters: %d %f %f %f %f %f %f %f",
    //   params_.n_,
    //   params_.velocitySigma_,
    //   params_.angleSigma_,
    //   params_.slipSigma_,
    //   params_.rangeSigma_,
    //   params_.bearingSigma_,
    //   params_.rangeWeightStd_,
    //   params_.bearingWeightStd_
    // );

    sigmaZ_ << params_.rangeSigma_, 0, 0, params_.bearingSigma_;

    const double& robotX = robotState(0);
    const double& robotY = robotState(1);
    const double& robotTheta = robotState(2);

    const double x = robotX + range * std::cos(wrapToPi(robotTheta) + wrapToPi(bearing));
    const double y = robotY + range * std::sin(wrapToPi(robotTheta) + wrapToPi(bearing));

    stateEstimate_ << x, y;

    stateCovariance_.setConstant(0.1);
}

void EKF::setMeasurementCovariance(Eigen::Matrix2d cov)
{
    sigmaZ_ = cov;
}

slamMeasurementVector_t EKF::measurementModel(slamStateVector_t robotState)
{
    double robotX = robotState(0);
    double robotY = robotState(1);

    double robotTheta = wrapToPi(robotState(2));

    double range = std::sqrt(std::pow(robotX - stateEstimate_(0), 2) +
                             std::pow(robotY - stateEstimate_(1), 2));

    double bearing = std::atan2(stateEstimate_(1) - robotY,
                                stateEstimate_(0) - robotX)
                    - robotTheta;

    bearing = wrapToPi(bearing);

    slamMeasurementVector_t output;
    output << range, bearing;
    return output;
}

Eigen::Matrix2d EKF::computeMeasurementJacobian(double range, double bearing, slamStateVector_t robotState)
{
    double robotX = robotState(0);
    double robotY = robotState(1);

    double xDist = stateEstimate_(0) - robotX;
    double yDist = stateEstimate_(1) - robotY;

    double denominator = std::pow(xDist, 2) + std::pow(yDist, 2);

    Eigen::Matrix2d jacobian;

    jacobian << xDist/std::sqrt(denominator), yDist/std::sqrt(denominator),
                -1*yDist/denominator, xDist/denominator;

    return jacobian;
}

// Returns the probability of getting the measurement
double EKF::correct(double range, double bearing, slamStateVector_t robotState)
{
  auto zHat = measurementModel(robotState);
  bearing = wrapToPi(bearing);

  auto H = computeMeasurementJacobian(range, bearing, robotState);

  Eigen::Matrix2d Q = H * stateCovariance_ * H.transpose() + sigmaZ_;

  Eigen::Matrix2d Qinv = Q.inverse();

  Eigen::Matrix2d K = stateCovariance_ * H.transpose() * Qinv;

  Eigen::Matrix<double, 2, 1> zt;
  zt << range, bearing;

  Eigen::Matrix<double, 2, 1> diff = zt - zHat;


  diff[1] = wrapToPi(diff[1]);

  double weightRange = gauss(zt(0), zHat(0), params_.rangeWeightStd_);
  double weightBearing = gauss(zt(1), zHat(1), params_.bearingWeightStd_);
  double weight = weightRange * weightBearing;

  stateEstimate_ += K*diff;
  stateCovariance_ = (Eigen::Matrix2d::Identity() - K*H)*stateCovariance_;

  // ROS_INFO("EKF correction:");
  // ROS_INFO("Range %f, ex range %f, range std %f, range weight %f", range, zHat(0), params_.rangeWeightStd_, weightRange);
  // ROS_INFO("Bearing %f, ex bearing %f, bearing std %f, bearing weight %f", bearing, zHat(1), params_.bearingWeightStd_, weightBearing);

  return weight;
}

}
