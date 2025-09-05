#pragma once

#include "Eigen/Dense"
#include "pose.hpp"
#include <vector>


class Path {
public:
  virtual Point2D getPoint(double t) = 0;
  virtual Point2D getDerivative(double t) = 0;
  virtual Point2D getSecondDerivative(double t) = 0;
  virtual double GetMaxT() const = 0;
  virtual ~Path() = default;
};

class CubicBezier : public Path {
public:
  CubicBezier(Point2D p0, Point2D p1, Point2D p2, Point2D p3);
  Point2D getPoint(double t) override;
  Point2D getDerivative(double t) override;
  Point2D getSecondDerivative(double t) override;
  double GetMaxT() const override;

private:
  Eigen::Matrix<double, 4, 2> points;

  Eigen::Matrix<double, 4, 4> matCoefficients;
  Eigen::Matrix<double, 3, 4> derivativeCoefficients;
  Eigen::Matrix<double, 2, 4> secondDerivativeCoefficients;
};

class MultiPath : public Path {
public:
  MultiPath(std::vector<Path *> paths) : paths(paths){};
  MultiPath(std::initializer_list<Path *> paths) {
    for (auto p : paths) {
      this->paths.push_back(p);
    }
  };
  Point2D getPoint(double t) override;
  Point2D getDerivative(double t) override;
  Point2D getSecondDerivative(double t) override;
  double GetMaxT() const override;

private:
  std::vector<Path *> paths;
};
