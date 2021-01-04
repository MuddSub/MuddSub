#include "ros/ros.h"
#include "slam/EvalFastSLAM.h"

bool evalFastSLAM(slam::EvalFastSLAM::Request  &params,
         slam::EvalFastSLAM::Response &res)
{
  res.pathRMS = 1.0;
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
