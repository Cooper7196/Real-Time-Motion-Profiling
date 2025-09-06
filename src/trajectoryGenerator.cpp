#include "trajectoryGenerator.hpp"
#include <iostream>
void TrajectoryGenerator::generateTrajectory(Path *path) {
  double t = 0;
  trajectory.clear();
  trajectory.push_back(Pose(path->getPoint(0).x, path->getPoint(0).y));
  Pose lastPose = trajectory.back();
  while (t < path->GetMaxT()) {
    double maxSpeed = kinematics->getMaxSpeed(path, lastPose, deltaD, t);
    Point2D derivative = path->getDerivative(t);
    double dt = deltaD /
                sqrt(derivative.x * derivative.x + derivative.y * derivative.y);
    lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, 0, maxSpeed, 0);
    trajectory.push_back(lastPose);
    t += dt;
  }
  int i = trajectory.size() - 1;
  t = path->GetMaxT();
  lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, 0, 0, 0);
  while (t > 0) {
    trajectory[i] = lastPose.velocity < trajectory[i].velocity
                  ? lastPose
                  : trajectory[i];
    double maxSpeed = kinematics->getMaxSpeed(path, lastPose, deltaD, t);
    Point2D derivative = path->getDerivative(t);
    double dt = deltaD /
                sqrt(derivative.x * derivative.x + derivative.y * derivative.y);
    lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, 0, maxSpeed, 0);
    t -= dt;
    i--;
  }
}

std::vector<Pose> TrajectoryGenerator::getTrajectory() { return trajectory; }