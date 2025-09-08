#include "kinematics.hpp"
#include "path.hpp"
#include "trajectoryGenerator.hpp"

#include <iostream>

int main()
{
  CubicBezier *testPath;
  testPath = new CubicBezier({12, 36}, {12, 60}, {36, 36}, {36, 60});

  Path *multiPath =
      new MultiPath({testPath, new CubicBezier({-36, -60}, {-36, -84},
                                               {-60, -60}, {-60, -84})});

  TrajectoryGenerator generator(new DifferentialKinematics(12, 75, 75, 0.4), 0.01);

  generator.generateTrajectory(multiPath);

  std::vector<Pose> trajectory = generator.getTrajectory();

  std::cout << std::endl;
}