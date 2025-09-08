#include "trajectoryGenerator.hpp"
#include <iostream>
void TrajectoryGenerator::generateTrajectory(Path *path)
{
  double t = 0;
  trajectory.clear();
  trajectory.push_back(Pose(path->getPoint(0).x, path->getPoint(0).y, atan2(path->getDerivative(0).y, path->getDerivative(0).x), 0, 0));
  Pose lastPose = trajectory.back();

  while (t < path->GetMaxT())
  {
    double maxSpeed = kinematics->getMaxSpeed(path, lastPose, deltaD, t);

    Point2D derivative = path->getDerivative(t);
    Point2D secondDerivative = path->getSecondDerivative(t);

    double dt = deltaD / sqrt(derivative.x * derivative.x + derivative.y * derivative.y);

    double curvature =
        (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) /
        pow(derivative.x * derivative.x + derivative.y * derivative.y, 1.5);

    lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, atan2(derivative.y, derivative.x), maxSpeed, maxSpeed * curvature);
    trajectory.push_back(lastPose);

    t += dt;
  }

  int i = trajectory.size() - 1;
  t = path->GetMaxT();
  lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, 0, 0, 0);
  while (t > 0)
  {
    trajectory[i] = lastPose.velocity < trajectory[i].velocity
                        ? lastPose
                        : trajectory[i];

    double maxSpeed = kinematics->getMaxSpeed(path, lastPose, deltaD, t);

    Point2D derivative = path->getDerivative(t);
    Point2D secondDerivative = path->getSecondDerivative(t);
    double curvature =
        (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) /
        pow(derivative.x * derivative.x + derivative.y * derivative.y, 1.5);

    lastPose = Pose(path->getPoint(t).x, path->getPoint(t).y, atan2(derivative.y, derivative.x), maxSpeed, maxSpeed * curvature);

    double dt = deltaD /
                sqrt(derivative.x * derivative.x + derivative.y * derivative.y);
    t -= dt;
    i--;
  }
}

std::vector<Pose> TrajectoryGenerator::getTrajectory() { return trajectory; }