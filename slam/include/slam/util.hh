#pragma once

#include <cmath>
#include <iterator>
#include <random>
#include <algorithm>

namespace MuddSub::SLAM
{

inline double wrapToPi(double val)
{
    auto th = std::fmod(val, 2*M_PI);

    if(th > M_PI)
      th -= 2*M_PI;
    else if(th <= -1*M_PI)
      th += 2*M_PI;
};

inline double gauss(double x, double mu, double stdDev)
{
  auto a = 1/(stdDev*std::sqrt(2*M_PI));
  auto b = -0.5/(std::pow(stdDev, 2));
  return a*std::exp(b*std::pow((x-mu), 2));
};


}
