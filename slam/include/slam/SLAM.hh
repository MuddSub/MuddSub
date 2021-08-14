#pragma once

#include <optional>
#include <vector>

namespace MuddSub::SLAM
{

template <typename T>
struct Measurement
{
  T value_;
  Sensor source_;
  double t_;
};

enum Sensor {DVL, FrontCamera, BottomCamera, Depth, Altimeter, IMU};

struct State
{
  SomePose pose;
  SomeMap map;
};

/* Inputs: Sensors
 * Outputs: Map, Odometry
 */
class SLAM
{

public:

  virtual bool update(const Frame& dataFrame);

  State getState() const
  {
    return state_;
  };

protected:

  struct Odometry
  {
    double vX_, vY_, vZ_;
  };

  struct CVDetection
  {
    std::optional<double> range_, horizontalBearing_, verticalBearing_;
  };

  struct Frame
  {
    int seq_;
    double t_;
    bool used_;
    std::vector<Measurement> measurements_;
  };


  State state_;

};
}
