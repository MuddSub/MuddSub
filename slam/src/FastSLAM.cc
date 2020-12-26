#include "slam/FastSLAM.hh"

namespace MuddSub::SLAM
{

FastSLAM::FastSLAM(std::string dataDirectory): data_(dataDirectory, robotID_, datasetID_)
{
  createParticles();
}

void FastSLAM::createParticles()
{
  slamStateVector_t initialState_ = {data_.robotData_[0].data_[0],
                                     data_.robotData_[0].data_[1],
                                     data_.robotData_[0].data_[2]};
  for(int i = 0; i < n_; ++i)
    particles_[i] = {n_, initialState_};
}

void FastSLAM::runFastSLAM()
{
  auto& robotData = data_.robotData_;

  double dt{0};
  double prevOdomTime = data_.robotOdometry_[0].time_;

  int dataLen = robotData.size();

  for(int i = 0; i < numSteps_; ++i)
  {
    KeyFrame keyFrame = robotData[0];
    robotData.erase(robotData.begin());

    double t = keyFrame.time_;

    if(keyFrame.type_ == "odometry")
    {
      dt = t - prevOdomTime;
      for(auto& p : particles_)
      {
        double thetaMeas = wrapToPi(data_.getCompass(t));
        p.propagateMotion(keyFrame.data_[0], thetaMeas, dt);
      }
    }
    else
    {
      auto& measurement = keyFrame.data_;
      double& subject = measurement[0];
      double& range = measurement[1];
      double& bearing = measurement[2];
      for(auto& p : particles_)
      {
        p.correct(t, subject, range, bearing);
      }

      auto getWeight = [](const Particle& p){return p.weight_;};

      std::array<double, n_> weights;

      std::transform(particles_.begin(), particles_.end(), weights.begin(), getWeight);

      double weightSum = std::accumulate(weights.begin(), weights.end(), 0);

      if(weightSum != 0.)
      {
        auto normalize = [weightSum](double& w){return w/weightSum;};
        std::for_each(weights.begin(), weights.end(), normalize);
      }
      else
      {
        std::cerr << "Warning: Weights were zero" << std::endl;
        std::fill(weights.begin(), weights.end(), 1./n_);
      }

      particles_ = resample(particles_, weights, true);

      stateLogs_.push_back(getStateAvg());
      timeSeries_.push_back(t);

      if(snapshotCounter_ == estimateSnapshotInterval_)
      {
        snapshotCounter_ = 0;
        std::array<Particle, n_> particleCopy = particles_;
        particleLogs_.push_back(particleCopy);
      }
      ++snapshotCounter_;
    }
  }
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
