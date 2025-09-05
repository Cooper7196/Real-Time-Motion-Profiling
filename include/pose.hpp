#pragma once

class Pose {
public:
  Pose();
  Pose(double x, double y);
  Pose(double x, double y, double theta);

  double distance(Pose pose);
  double angle(Pose pose);

  double x;
  double y;
  double theta;
};

class Point2D {
public:
  Point2D() : x(0), y(0) {}
  Point2D(double x, double y) : x(x), y(y) {}
  double x;
  double y;
};