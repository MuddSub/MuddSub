#pragma once

#include <unordered_map>
#include <string>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <array>

#include "slam/Types.hh"

namespace MuddSub::SLAM
{

class DataLoader
{

public:
  DataLoader(const std::string& dataDirectory, const int& robotId, const int& datasetId);

  double getCompass(double t) const;

  double getXTruth(double t) const;
  double getYTruth(double t) const;

  inline slamStateVector_t getInitialState()
  {
    return {robotGroundTruth_[0].data_[0], robotGroundTruth_[0].data_[1], robotGroundTruth_[0].data_[2]};
  };

private:

  void loadBarcodes();
  void loadGroundtruth();
  void loadMeasurements();
  void loadOdometry();
  void loadMap();

  void mergeRobotData();


  std::vector<std::vector<double>> parseFileToVectors(const std::string& path);


  std::string barcodePath_;
  std::string groundtruthPath_;
  std::string measurementPath_;
  std::string odometryPath_;
  std::string mapPath_;

  std::unordered_map<int, std::pair<int, int>> groundTruthMap_;

  std::unordered_map<int, int> barcodeMap_;

  std::vector<KeyFrame> robotData_;

  std::vector<KeyFrame> robotGroundTruth_;
  std::vector<KeyFrame> robotMeasurement_;
  std::vector<KeyFrame> robotOdometry_;

  friend class FastSLAM;

};


}
