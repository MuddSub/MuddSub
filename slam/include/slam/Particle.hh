#pragma once
#include <unordered_map>
#include "slam/EKF.hh"


#namespace MuddSub::SLAM
{

class Particle
{

private:
  std::unordered_map<int, EKF> landmarkEKFs;

};

};
