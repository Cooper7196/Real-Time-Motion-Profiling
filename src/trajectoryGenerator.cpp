#include "trajectoryGenerator.hpp"

void TrajectoryGenerator::generateTrajectory(Path *path) {
  double t = 0;
  trajectory.clear();
  trajectory.push_back(Pose(path->getPoint(0).x, path->getPoint(0).y, 0));
  Pose lastPose = trajectory.back();
  while (t < path->GetMaxT()) {
    double maxSpeed = kinematics->getMaxSpeed(path, t);
    double 
    Point2D derivative = path->getDerivative(t);
    double dt = deltaD /
                sqrt(derivative.x * derivative.x + derivative.y * derivative.y);
    t += dt;
  }
}