#include "slam/DataLoader.hh"

namespace MuddSub::SLAM
{

DataLoader::DataLoader(const int& datasetId, const int& robotId)
{
  std::string slamPath = ros::package::getPath("slam");
  std::string directoryPath = slamPath + "/datasets/MRCLAM_Dataset" + std::to_string(datasetId);

  std::string robotPrefix = directoryPath + "/Robot" + std::to_string(robotId) + "_";

  groundtruthPath_ = robotPrefix + "Groundtruth.dat";
  measurementPath_ = robotPrefix + "Measurement.dat";
  odometryPath_ = robotPrefix + "Odometry.dat";

  barcodePath_ = directoryPath + "/Barcodes.dat";
  mapPath_ = directoryPath + "/Landmark_Groundtruth.dat";

  double begin = ros::Time::now().toSec();
  loadBarcodes();
  loadGroundtruth();
  loadMeasurements();
  loadOdometry();
  loadMap();
  mergeRobotData();
  std::cout << "Time to load: " << ros::Time::now().toSec()-begin << std::endl;

  ROS_INFO("Done building dataset");
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
  bool first{true};

  for(const auto& line : groundtruthData)
  {
    KeyFrame frame;
    frame.type_ = "groundtruth";
    if(first)
    {
      first = false;
      startTime_ = line[0];
    }

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
      ROS_INFO("Barcode %s not recognized. Skipping %f", line[1]);
      continue;
    }
    int subject = barcodeMap_[line[1]];
    frame.data_ = {subject, line[2], line[3]};

    robotMeasurement_.push_back(frame);
  }

  // dumpVectorToFile("Measurements.txt", robotMeasurement_);
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
  // dumpVectorToFile("Odometry.txt", robotOdometry_);

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

  auto measIt = robotMeasurement_.begin();
  auto odomIt = robotOdometry_.begin();
  while(measIt != robotMeasurement_.end() && odomIt != robotOdometry_.end())
  {
    if(*measIt < *odomIt)
    {
      robotData_.push_back(*measIt);
      ++measIt;
    }
    else
    {
      robotData_.push_back(*odomIt);
      ++odomIt;
    }
  }

  if(measIt == robotMeasurement_.end())
  {
    while(odomIt != robotOdometry_.end())
    {
      robotData_.push_back(*odomIt);
      ++odomIt;
    }
  }
  else
  {
    while(measIt != robotMeasurement_.end())
    {
      robotData_.push_back(*measIt);
      ++measIt;
    }
  }

  // dumpVectorToFile("Merged.txt", robotData_);

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
      std::string number;
      std::stringstream ssword(word);
      while(std::getline(ssword, number, '\t'))
      {
        if(number != "")
        {
          line.push_back(std::stod(number));
        }
      }
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


void DataLoader::dumpVectorToFile(const std::string& fname, std::vector<KeyFrame> data)
{
  std::ofstream outfile;
  outfile.open(fname);
  for(auto& frame : data)
  {
    outfile << frame.type_ << ", " << frame.time_ << ", ";
    for(auto& d : frame.data_)
    {
      outfile << d << ", ";
    }
    outfile << std::endl;
  }
  outfile.close();
}

}
