#pragma once
#include "kinematics.hpp"
#include "path.hpp"

#include <vector>

class TrajectoryGenerator
{
public:
  TrajectoryGenerator(Kinematics *kinematics, double deltaD)
      : kinematics(kinematics), deltaD(deltaD) {};
  void generateTrajectory(Path *path);
  std::vector<Pose> getTrajectory();

private:
  Kinematics *kinematics;
  double deltaD;
  std::vector<Pose> trajectory;
};