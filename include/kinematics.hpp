#pragma once
#include "path.hpp"
#include <vector>

class Kinematics
{
public:
  virtual double getMaxSpeed(Path *path, Pose lastPose, double deltaD, double t) const = 0;
  virtual std::vector<double> getWheelVelocities(double linearVel,
                                                 double angularVel) const = 0;
  virtual ~Kinematics() = default;
};

class DifferentialKinematics : public Kinematics
{
public:
  DifferentialKinematics(double trackWidth, double maxVel, double maxAccel,
                         double friction)
      : trackWidth(trackWidth), maxVel(maxVel), maxAccel(maxAccel),
        friction(friction) {}
  double getMaxSpeed(Path *path, Pose lastPose, double deltaD, double t) const override;
  std::vector<double> getWheelVelocities(double linearVel,
                                         double angularVel) const override;

private:
  double trackWidth;
  double maxVel;
  double maxAccel;
  double friction;
};