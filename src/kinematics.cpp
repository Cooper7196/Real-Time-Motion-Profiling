#include "kinematics.hpp"
#include "pose.hpp"

double DifferentialKinematics::getMaxSpeed(Path *path, Pose curPose,
                                           double t) const {
  Point2D derivative = path->getDerivative(t);
  Point2D secondDerivative = path->getSecondDerivative(t);

  double curvature =
      (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) /
      pow(derivative.x * derivative.x + derivative.y * derivative.y, 1.5);

  if (curvature == 0) {
    return maxVel;
  }

  double maxSpeedCurvature = 2 * maxVel / (2 + trackWidth * fabs(curvature));

  double maxSpeedFriction = sqrt(friction * maxAccel / (fabs(curvature)));

  return std::min({maxSpeedCurvature, maxSpeedFriction, this->maxVel});
}

std::vector<double>
DifferentialKinematics::getWheelVelocities(double linearVel,
                                           double angularVel) const {
  double leftWheelVel = linearVel - (angularVel * trackWidth / 2);
  double rightWheelVel = linearVel + (angularVel * trackWidth / 2);
  return {leftWheelVel, rightWheelVel};
}