#include "slam/DataLoader.hh"

namespace MuddSub::SLAM
{

DataLoader::DataLoader(const std::string& dataDirectory, const int& robotId, const int& datasetId)
{
  std::string slamPath = ros::package::getPath("slam");
  std::string directoryPath = slamPath + "/datasets/MRCLAM_Dataset" + std::to_string(datasetId);

  std::string robotPrefix = directoryPath + "/Robot" + std::to_string(robotId) + "_";

  barcodePath_ = robotPrefix + "Barcodes.dat";
  groundtruthPath_ = robotPrefix + "Groundtruth.dat";
  measurementPath_ = robotPrefix + "Measurement.dat";
  odometryPath_ = robotPrefix + "Odometry.dat";

  mapPath_ = directoryPath + "/Landmark_Groundtruth.dat";


  loadBarcodes();
  loadGroundtruth();
  loadMeasurements();
  loadOdometry();
  loadMap();

  mergeRobotData();
}

void DataLoader::loadBarcodes()
{
  auto barcodeData = parseFileToVectors(barcodePath_);

  for(const auto& line : barcodeData)
  {
    // Map barcode to subject
    int barcode = line[1];
    int subject = line[0];
    barcodeMap_[barcode] = subject;
  }
}

void DataLoader::loadGroundtruth()
{
  auto groundtruthData = parseFileToVectors(groundtruthPath_);

  for(const auto& line : groundtruthData)
  {
    KeyFrame frame;
    frame.type_ = "groundtruth";
    frame.time_ = line[0];
    frame.data_ = {line[1], line[2], line[3]};
    robotGroundTruth_.push_back(frame);
  }
}

void DataLoader::loadMeasurements()
{
  auto measurementData = parseFileToVectors(measurementPath_);

  for(const auto& line : measurementData)
  {
    KeyFrame frame;
    frame.type_ = "measurement";
    frame.time_ = line[0];
    if(barcodeMap_.count(line[1]) == 0)
    {
      std::cout << "Barcode " << line[1] << " not recognized. Skipping." << std::endl;
      continue;
    }
    int subject = barcodeMap_[line[1]];
    frame.data_ = {subject, line[2], line[3]};

    robotMeasurement_.push_back(frame);
  }
}

void DataLoader::loadOdometry()
{
  auto odometryData = parseFileToVectors(odometryPath_);

  for(const auto& line : odometryData)
  {
    KeyFrame frame;

    frame.type_ = "odometry";
    frame.time_ = line[0];
    frame.data_ = {line[1], line[2]};

    robotOdometry_.push_back(frame);
  }
}

void DataLoader::loadMap()
{
  auto mapData = parseFileToVectors(mapPath_);

  for(const auto& line : mapData)
  {
    groundTruthMap_[line[0]] = {line[1], line[2]};
  }
}

void DataLoader::mergeRobotData()
{
  robotData_ = std::vector<KeyFrame>();

  // Merge measurement and odometry into single stream, sorted by time.
  std::merge(robotMeasurement_.begin(), robotMeasurement_.end(),
             robotOdometry_.begin(), robotOdometry_.end(),
             robotData_.end());
}

std::vector<std::vector<double>> DataLoader::parseFileToVectors(const std::string& path)
{
  std::ifstream file(path);
  std::string text;

  int i{0};

  std::vector<std::vector<double>> output;

  while(std::getline(file, text))
  {
    if(i < 4)
    {
      ++i;
      continue;
    }

    std::vector<double> line;

    std::stringstream ss(text);
    std::string word;

    int rowWordCounter = 0;

    int subject, barcode;

    while(std::getline(ss, word, ' '))
    {
      if(word != "")
        line.push_back(std::stod(word));
    }

    output.push_back(line);
  }
  return output;
}

double DataLoader::getCompass(double t) const
{
  auto truthIt = std::lower_bound(robotGroundTruth_.begin(), robotGroundTruth_.end(), t);
  return truthIt->data_[2];
}

double DataLoader::getXTruth(double t) const
{
  auto truthIt = std::lower_bound(robotGroundTruth_.begin(), robotGroundTruth_.end(),t);
  return truthIt->data_[0];
}

double DataLoader::getYTruth(double t) const
{
  auto truthIt = std::lower_bound(robotGroundTruth_.begin(), robotGroundTruth_.end(),t);
  return truthIt->data_[1];
}



}
