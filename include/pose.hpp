#pragma once

class Pose
{
public:
  Pose();
  Pose(double x, double y) : x(x), y(y), theta(0), velocity(0), angularVelocity(0) {};
  Pose(double x, double y, double theta) : x(x), y(y), theta(theta), velocity(0), angularVelocity(0) {};
  Pose(double x, double y, double theta, double velocity, double angularVelocity) : x(x), y(y), theta(theta), velocity(velocity), angularVelocity(angularVelocity) {};
  double distance(Pose pose);
  double angle(Pose pose);

  double x;
  double y;
  double theta;
  double velocity;
  double angularVelocity;
};

class Point2D
{
public:
  Point2D() : x(0), y(0) {}
  Point2D(double x, double y) : x(x), y(y) {}
  double x;
  double y;
};