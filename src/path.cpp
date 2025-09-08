#include "path.hpp"

CubicBezier::CubicBezier(Point2D p0, Point2D p1, Point2D p2, Point2D p3)
{
  this->points << p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y;

  this->matCoefficients << -1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0;
  this->derivativeCoefficients << -3, 9, -9, 3, 6, -12, 6, 0, -3, 3, 0, 0;
  this->secondDerivativeCoefficients << -6, 18, -18, 6, 6, -12, 6, 0;
}

Point2D CubicBezier::getPoint(double t)
{
  Eigen::Matrix<double, 1, 4> T;
  T << t * t * t, t * t, t, 1;
  auto result = (T * this->matCoefficients) * this->points;
  return {result(0), result(1)};
}

Point2D CubicBezier::getDerivative(double t)
{
  Eigen::Matrix<double, 1, 3> T;
  T << t * t, t, 1;
  Eigen::Matrix<double, 1, 2> result =
      T * this->derivativeCoefficients * this->points;
  return {result(0), result(1)};
}

Point2D CubicBezier::getSecondDerivative(double t)
{
  Eigen::Matrix<double, 1, 2> T;
  T << t, 1;
  Eigen::Matrix<double, 1, 2> result =
      T * this->secondDerivativeCoefficients * this->points;
  return {result(0), result(1)};
}

double CubicBezier::GetMaxT() const { return 1.0; }

Point2D MultiPath::getPoint(double t)
{
  double totalT = 0;
  for (auto path : paths)
  {
    if (t <= totalT + path->GetMaxT())
    {
      return path->getPoint(t - totalT);
    }
    totalT += path->GetMaxT();
  }
  return paths.back()->getPoint(paths.back()->GetMaxT());
}

Point2D MultiPath::getDerivative(double t)
{
  double totalT = 0;
  for (auto path : paths)
  {
    if (t <= totalT + path->GetMaxT())
    {
      return path->getDerivative(t - totalT);
    }
    totalT += path->GetMaxT();
  }
  return paths.back()->getDerivative(paths.back()->GetMaxT());
}

Point2D MultiPath::getSecondDerivative(double t)
{
  double totalT = 0;
  for (auto path : paths)
  {
    if (t <= totalT + path->GetMaxT())
    {
      return path->getSecondDerivative(t - totalT);
    }
    totalT += path->GetMaxT();
  }
  return paths.back()->getSecondDerivative(paths.back()->GetMaxT());
}

double MultiPath::GetMaxT() const
{
  double totalT = 0;
  for (auto path : paths)
  {
    totalT += path->GetMaxT();
  }
  return totalT;
}