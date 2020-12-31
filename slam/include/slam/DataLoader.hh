#pragma once

#include <unordered_map>
#include <string>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <array>
#include <ros/ros.h>

#include "slam/Types.hh"

namespace MuddSub::SLAM
{

class DataLoader
{

public:
  DataLoader(const int& datasetId, const int& robotId);

  double getCompass(double t) const;

  double getXTruth(double t) const;
  double getYTruth(double t) const;

  inline slamStateVector_t getInitialState()
  {
    return {robotGroundTruth_[0].data_[0], robotGroundTruth_[0].data_[1], robotGroundTruth_[0].data_[2]};
  };

private:

  void loadBarcodes();

  // Note: Ground truth must be called before the other four below.
  void loadGroundtruth();
  void loadMeasurements();
  void loadOdometry();
  void loadMap();

  void mergeRobotData();

  void dumpVectorToFile(const std::string& fname, std::vector<KeyFrame> data);

  std::vector<std::vector<double>> parseFileToVectors(const std::string& path);


  std::string barcodePath_;
  std::string groundtruthPath_;
  std::string measurementPath_;
  std::string odometryPath_;
  std::string mapPath_;

  map_t groundTruthMap_;

  std::unordered_map<int, int> barcodeMap_;

  std::vector<KeyFrame> robotData_;

  std::vector<KeyFrame> robotGroundTruth_;
  std::vector<KeyFrame> robotMeasurement_;
  std::vector<KeyFrame> robotOdometry_;

  double startTime_{0};

  friend class FastSLAM;

};


}
