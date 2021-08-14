#pragma once

#include <cmath>
#include <iterator>
#include <random>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>

namespace MuddSub::SLAM
{

inline double wrapToPi(double val)
{
  auto th = std::fmod(val, 2*M_PI);

  if(th > M_PI)
    th -= 2*M_PI;
  else if(th < -1*M_PI)
    th += 2*M_PI;

  return th;
};

inline double gauss(double x, double mu, double stdDev, double scale=1)
{
  double result = (1./(stdDev*std::sqrt(2*M_PI))) *std::exp(-1./2.*std::pow((x-mu)/stdDev, 2));
  return result*scale;
};

}
