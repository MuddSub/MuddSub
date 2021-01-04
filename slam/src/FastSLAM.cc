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
  for(int i = 0; i < n_; ++i)
  {
    particles_[i] = {n_, initialState_};
  }
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

  tqdm bar;
  for(auto& keyFrame : robotData)
  {
    if(!ros::ok() || i == numSteps_) break;
    bar.progress(i, numSteps_);
    double t = keyFrame.time_;

    if(keyFrame.type_ == "odometry")
    {
      dt = t - prevOdomTime;
      prevOdomTime = t;
      double thetaMeas = wrapToPi(data_.getCompass(t));
      // ROS_INFO("Propagate motion step");
      for(auto& p : particles_)
        p.propagateMotion(keyFrame.data_[0], thetaMeas, dt);
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
      std::array<double, n_> weights;

      int j = 0;
      for(auto& p : particles_)
      {
        weights[j] = p.correct(t, subject, range, bearing);
        ++j;
      }

      double weightSum = 0;
      for(int wIt = 0; wIt < n_; ++wIt)
      {
        weightSum += weights[wIt];
      }

      if(weightSum != 0.)
      {
        for(int wIt = 0; wIt < n_; ++wIt)
        {
          weights[wIt] /= weightSum;
        }
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

  std::vector<State> robotStates;
  std::vector<State> groundTruthStates;

  for(int i=0; i<xTruth.size(); ++i){
  	State robotState, groundTruthState;
	groundTruthState.x_ = xTruth[i];
	groundTruthState.y_ = yTruth[i];
	robotState.x_ = robotX[i];
	robotState.y_ = robotY[i];
	groundTruthStates.push_back(groundTruthState);
	robotStates.push_back(robotState);
  }

  plt::plot(robotX, robotY);
  plt::plot(xTruth, yTruth);
  plt::scatter(xLandmarks, yLandmarks, 8);
  plt::scatter(xLandmarkTruth, yLandmarkTruth, 8);
  plt::show();

  auto rms = FastSLAM::euclidRMS(groundTruthStates,robotStates);
  std::cout<<"\n";
  std::cout<<"number of particles "<<n_<<"\n";
  std::cout<<"number of steps "<<numSteps_<<"\n";
  std::cout<<"rms "<<rms<<"\n";
}

std::array<Particle, FastSLAM::n_> FastSLAM::resample(const std::array<Particle, n_>& input,
                                                      const std::array<double, n_>& weights,
                                                      bool normalized)
{
  assert(*std::min_element(weights.begin(), weights.end()) >= 0.);

  std::array<double, n_> normalizedWeights = weights;
  if(!normalized)
  {
    double weightSum = 0;
    for(int wIt = 0; wIt < n_; ++wIt)
      weightSum += weights[wIt];

    if(weightSum != 1.)
    {
      for(int wIt = 0; wIt < n_; ++wIt)
        normalizedWeights[wIt] = weights[wIt]/weightSum;
    }
  }
  else
    normalizedWeights = weights;
  std::random_device r;
  std::default_random_engine generator = std::default_random_engine{r()};
  std::uniform_real_distribution<double> uniform(0., 1.);

  std::array<Particle, n_> output;

  for(int i = 0; i < n_; ++i)
  {
    double rand = uniform(generator);
    double runningWeight = 0;
    auto selectorIt = input.begin();

    for(auto w : normalizedWeights)
    {
      runningWeight += w;
      if(rand < runningWeight)
      {
        output[i] = *selectorIt;
        break;
      }
	    ++selectorIt;
    }
  }

  // printf("Resampled. New particles are: ");
  // for(int i = 0; i < n_; ++i)
  // {
  //   printf("%d, ", output[i].id_);
  // }
  // printf("\n");

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
  std::vector<float> errors;

  auto estimateIt = stateEstimates.begin();
  for(auto truth : stateTruths)
  {
    auto dist = euclid(truth.x_, estimateIt->x_,
                       truth.y_, estimateIt->y_);
    errors.push_back(dist);
    ++estimateIt;
  }
  return std::sqrt(std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0)/errors.size());
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SLAM");
  ros::NodeHandle nh;
  MuddSub::SLAM::FastSLAM slam{1, 1};
  slam.runFastSLAM();

}
