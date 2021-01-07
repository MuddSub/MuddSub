#include "slam/FastSLAM.hh"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SLAM");
  ros::NodeHandle nh;
  MuddSub::SLAM::DataLoader data{1, 1};
  MuddSub::SLAM::FastSLAM slam{1, 1, data};
  slam.runFastSLAM(true);
}
