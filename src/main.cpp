#include "kinematics.hpp"
#include "path.hpp"
#include "trajectoryGenerator.hpp"

#include <iostream>

#define M_PI 3.14159265358979323846

int main() {

  CubicBezier *testPath;
  testPath = new CubicBezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60});

  Path *multiPath =
      new MultiPath({testPath, new CubicBezier({-36, -60}, {-36, -84},
                                               {-60, -60}, {-60, -84})});

  // for (int i = 0; i <= multiPath->GetMaxT() * 100; i++) {
  //   std::cout << multiPath->getPoint(i / 100.0).x << ","
  //             << multiPath->getPoint(i / 100.0).y << "|";
  // }
  // std::cout << std::endl;

  TrajectoryGenerator generator(new DifferentialKinematics(12, 100, 100, 1), 1);
  generator.generateTrajectory(multiPath);
}