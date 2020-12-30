#include "slam/FastSLAM.hh"

namespace plt = matplotlibcpp;

namespace MuddSub::SLAM
{

FastSLAM::FastSLAM(const int& datasetId, const int& robotId):
  datasetID_(datasetId), robotID_(robotId), data_(robotID_, datasetID_)
{
  createParticles();
}

void FastSLAM::createParticles()
{
  slamStateVector_t initialState_ = {data_.robotGroundTruth_[0].data_[0],
                                     data_.robotGroundTruth_[0].data_[1],
                                     data_.robotGroundTruth_[0].data_[2]};
  std::cerr << "Initial: " << initialState_ << std::endl;
  for(int i = 0; i < n_; ++i)
    particles_[i] = {n_, initialState_};
}

void FastSLAM::runFastSLAM()
{
  auto& robotData = data_.robotData_;

  double dt{0};
  double prevOdomTime = data_.robotOdometry_[0].time_;

  int dataLen = robotData.size();

  int i{0};
  double start = ros::Time::now().toSec();

  int numCorrect{0};

  for(auto& keyFrame : robotData)
  {
    if(!ros::ok() || i == numSteps_) break;

    ROS_INFO("===== Iteration %d ===== ", i);

    double t = keyFrame.time_;

    if(keyFrame.type_ == "odometry")
    {
      dt = t - prevOdomTime;
      prevOdomTime = t;
      double thetaMeas = wrapToPi(data_.getCompass(t));
      for(auto& p : particles_)
      {
        p.propagateMotion(keyFrame.data_[0], thetaMeas, dt);
        double xTruth{data_.getXTruth(t)}, yTruth{data_.getYTruth(t)};
        p.robotState_ = {xTruth, yTruth, thetaMeas}; 
        // std::stringstream ss;
        // ss << p.robotState_;
        // auto stateString = ss.str();
        // std::replace(stateString.begin(), stateString.end(), '\n', ',');
        // ROS_INFO("I: %d, T: %.9f", i, t);
      }
    }
    else
    {
      // ++i;//remove
      // continue;
      numCorrect += 1;

      auto& measurement = keyFrame.data_;
      int subject = static_cast<int>(measurement[0]);

      if(subject < 5)
      {
        ++i;
        continue;
      }

      double& range = measurement[1];
      double& bearing = measurement[2];
      for(auto& p : particles_)
      {
        p.correct(t, subject, range, bearing);
      }

      auto getWeight = [](const Particle& p){return p.weight_;};

      std::array<double, n_> weights;

      for(int w = 0; w < particles_.size(); ++w)
        weights[w] = getWeight(particles_[w]);


      double weightSum = std::accumulate(weights.begin(), weights.end(), 0);
      if(weightSum != 0.)
      {
        ROS_INFO("Weight sum %f", weightSum);
        auto normalize = [weightSum](double& w){return w/weightSum;};
        std::for_each(weights.begin(), weights.end(), normalize);
      }
      else
      {
        ROS_WARN("Weights were zero %f", weightSum);
        std::fill(weights.begin(), weights.end(), 1./n_);
      }

      particles_ = resample(particles_, weights, true);
    }
    stateLogs_.push_back(getStateAvg());
    timeSeries_.push_back(t);

    if(snapshotCounter_ == estimateSnapshotInterval_)
    {
      snapshotCounter_ = 0;
      std::array<Particle, n_> particleCopy = particles_;
      particleLogs_.push_back(particleCopy);
    }
    ++snapshotCounter_;
    ++i;
  }
  ROS_INFO("Total time: %f", ros::Time::now().toSec() - start);


  std::vector<double> xLandmarks;
  std::vector<double> yLandmarks;
  std::vector<double> robotX;
  std::vector<double> robotY;

  std::vector<double> xTruth;
  std::vector<double> yTruth;

  const auto& lastLog = stateLogs_.back();
  for(const auto& landmark : lastLog.map_)
  {
    xLandmarks.push_back(landmark.second.first);
    yLandmarks.push_back(landmark.second.second);
  }

  auto timeSeriesIt = timeSeries_.begin();
  for(const auto& log : stateLogs_)
  {
    double t = *timeSeriesIt;
    robotX.push_back(log.x_);
    robotY.push_back(log.y_);
    xTruth.push_back(data_.getXTruth(t));
    yTruth.push_back(data_.getYTruth(t));
    ++timeSeriesIt;
  }

  const auto& truthMap = data_.groundTruthMap_;

  std::vector<double> xLandmarkTruth;
  std::vector<double> yLandmarkTruth;

  for(const auto& landmark : truthMap)
  {
    xLandmarkTruth.push_back(landmark.second.first);
    yLandmarkTruth.push_back(landmark.second.second);
  }

  plt::plot(robotX, robotY);
  plt::plot(xTruth, yTruth);
  plt::scatter(xLandmarks, yLandmarks, 8);
  plt::scatter(xLandmarkTruth, yLandmarkTruth, 8);
  plt::show();
}

std::array<Particle, FastSLAM::n_> FastSLAM::resample(const std::array<Particle, n_>& input,
                                                      const std::array<double, n_>& weights,
                                                      bool normalized)
{
  assert(*std::min_element(weights.begin(), weights.end()) >= 0.);

  if(!normalized)
  {
    double weightSum = std::accumulate(weights.begin(), weights.end(), 0);
    if(weightSum != 1.)
    {
      auto normalize = [weightSum](const double& w){return w/weightSum;};
      std::for_each(weights.begin(), weights.end(), normalize);
    }
  }

  std::default_random_engine generator;
  std::uniform_real_distribution<double> uniform(0., 1.);

  std::array<Particle, n_> output;

  for(int i = 0; i < n_; ++i)
  {
    double rand = uniform(generator);

    double runningWeight = 0;

    auto selectorIt = input.begin();

    for(auto w : weights)
    {
      runningWeight += w;
      if(rand < runningWeight)
      {
        output[i] = *selectorIt;
        break;
      }
    }
  }

  return output;
};

State FastSLAM::getStateAvg()
{
  double xSum{0}, ySum{0}, thSum{0};

  for(auto p : particles_)
  {
    xSum += p.robotState_[0];
    ySum += p.robotState_[1];
    thSum += p.robotState_[2];
  }

  double xAvg = xSum/n_;
  double yAvg = ySum/n_;
  double thAvg = thSum/n_;

  State output;

  output.x_ = xAvg;
  output.y_ = yAvg;
  output.theta_ = thAvg;

  for(const auto& landmark : particles_[0].landmarkEKFs_)
  {
    int subject = landmark.first;

    double xLandmarkSum{0}, yLandmarkSum{0};
    for(auto& p : particles_)
    {
      xLandmarkSum += p.landmarkEKFs_[subject]->stateEstimate_[0];
      yLandmarkSum += p.landmarkEKFs_[subject]->stateEstimate_[1];
    }
    output.map_[subject] = {xLandmarkSum/n_, yLandmarkSum/n_};
  }

  return output;
}

double FastSLAM::euclidRMS(std::vector<State> stateTruths, std::vector<State> stateEstimates)
{
  std::vector<double> errors;

  auto estimateIt = stateEstimates.begin();
  for(auto truth : stateTruths)
  {
    auto dist = euclid(truth.x_, estimateIt->x_,
                       truth.y_, estimateIt->y_);
    errors.push_back(dist);
    ++estimateIt;
  }
  return std::sqrt(std::inner_product(errors.begin(), errors.end(), errors.begin(), 0));
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SLAM");
  ros::NodeHandle nh;
  MuddSub::SLAM::FastSLAM slam{1, 1};
  slam.runFastSLAM();

}
