#include "slam/FastSLAM.hh"

namespace plt = matplotlibcpp;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SLAM");
  ros::NodeHandle nh;

  // Set parameters. Optimal parameters obtained from simualted annealing
  MuddSub::SLAM::Parameters params_ = {
    .n_ = (long int)50,
    .velocitySigma_ = (double)0.0733517,
    .angleSigma_ = (double)0.00675702,
    .slipSigma_ = (double)0.1442912,
    .rangeSigma_ = (double)0.17709347,
    .bearingSigma_ = (double)0.05907082,
    .rangeWeightStd_ = (double)0.14237548,
    .bearingWeightStd_ = (double)0.01817904
  };

  MuddSub::SLAM::DataLoader data{1, 1};

  // Initialize counters to keep track of trials
  int its = 100;
  std::vector<float> rmsHist{};
  float acc = 0;

  // Evaluate FastSLAM and create each subplot
  for (int i = 0; i < its; i++) {
    // plt::figure();
    MuddSub::SLAM::FastSLAM slam{1, 1, data, params_};
    float rms = slam.runFastSLAM(false);
    rmsHist.push_back(rms);
    printf("%d RMS: %f\n", i, rms);
    acc += rms;
  }

  // Calculate and print average
  float avg = acc / (float)its;
  printf("\n\n\n");
  printf("Number of iterations: %d\n", its);
  printf("RMS Average: %f\n", avg);
  acc = 0;

  // Calculate and print variance
  for (int i = 0; i < its; i++) {
    float error = rmsHist[i] - avg;
    acc += error * error;
  }
  printf("RMS Variance: %f\n", acc / (float)its);

  // Show plots
  // plt::show();
}
