#include "ros/ros.h"
#include "slam/EvalFastSLAM.h"
#include "slam/FastSLAM.hh"

bool evalFastSLAM(slam::EvalFastSLAM::Request  &params,
         slam::EvalFastSLAM::Response &res)
{
  ROS_INFO("request: %ld, %f, %f, %f, %f, %f, %f, %f",
    (long int)params.n,
    (double)params.velocitySigma,
    (double)params.angleSigma,
    (double)params.slipSigma,
    (double)params.rangeSigma,
    (double)params.bearingSigma,
    (double)params.rangeWeightStd,
    (double)params.bearingWeightStd
  );

  MuddSub::SLAM::Parameters params_ = {
    .n_ = (long int)params.n,
    .velocitySigma_ = (double)params.velocitySigma,
    .angleSigma_ = (double)params.angleSigma,
    .slipSigma_ = (double)params.slipSigma,
    .rangeSigma_ = (double)params.rangeSigma,
    .bearingSigma_ = (double)params.bearingSigma,
    .rangeWeightStd_ = (double)params.rangeWeightStd,
    .bearingWeightStd_ = (double)params.bearingWeightStd
  };
  MuddSub::SLAM::FastSLAM slam{1, 1, params_};
  // MuddSub::SLAM::FastSLAM slam = MuddSub::SLAM::FastSLAM(1, 1, params_);
  res.pathRMS = slam.runFastSLAM(false);
  // res.pathRMS = 1.0;

  ROS_INFO("sending back response: [%f]", (double)res.pathRMS);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SLAMEvalServer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("evalFastSLAM", evalFastSLAM);
  ROS_INFO("Ready to evaluate FastSLAM.");
  ros::spin();

  return 0;
}
