# Real-Time Motion Profiling for VEX Robotics

[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A high-performance, real-time motion profiling library designed specifically for VEX Robotics competitions. This library generates smooth, kinematics-constrained trajectories for autonomous robot movement.

## Features

- **Real-time trajectory generation** optimized for rapid regeneration in under 5ms
- **Differential drive kinematics** with configurable robot parameters
- **Multi-path support** for complex autonomous routines
- **Physics-based constraints** including friction, acceleration, and velocity limits

## Architecture

### Core Components

#### Path Generation (`path.hpp/cpp`)
- **CubicBezier**: Implements smooth parametric curves using control points
- **MultiPath**: Concatenates multiple path segments for complex trajectories
- **Mathematical derivatives**: Calculated first and second derivatives using Eigen

#### Kinematics Engine (`kinematics.hpp/cpp`)
- **DifferentialKinematics**: Models tank drive/differential drive robots
- **Configurable parameters**: Track width, maximum velocity, acceleration, and friction coefficients
- **Wheel velocity calculations**: Converts linear and angular velocities to individual wheel speeds for use on a robot

#### Trajectory Generator (`trajectoryGenerator.hpp/cpp`)
- **Double pass**: Calculates maximum safe velocities considering robot constraints
- **Time-optimal profiling**: Minimizes trajectory execution time while respecting physical limits

## Quick Start

### Prerequisites
- C++17 compatible compiler (GCC 7.0+ recommended)
- Eigen3 library (included in `include/Eigen/`)
- Make build system

### Building the Project

```bash
# Clone the repository
git clone https://github.com/Cooper7196/Real-Time-Motion-Profiling.git
cd Real-Time-Motion-Profiling

# Build the project
make

# Run the example
./main
```

### Basic Usage

```cpp
#include "kinematics.hpp"
#include "path.hpp"
#include "trajectoryGenerator.hpp"

int main() {
    // Define robot parameters
    double trackWidth = 12.0;    // inches
    double maxVel = 75.0;        // inches/second  
    double maxAccel = 75.0;      // inches/second²
    double friction = 0.4;       // coefficient of friction
    
    // Create kinematics model
    auto kinematics = new DifferentialKinematics(trackWidth, maxVel, maxAccel, friction);
    
    // Define path using control points
    auto path = new CubicBezier({0, 0}, {24, 0}, {24, 24}, {48, 24});
    
    // Generate trajectory
    TrajectoryGenerator generator(kinematics, 0.01); // 1cm resolution
    generator.generateTrajectory(path);
    
    // Get trajectory points
    std::vector<Pose> trajectory = generator.getTrajectory();
    
    // Use trajectory for robot control...
    return 0;
}
```

## Learning Resources

### Motion Profiling.pdf
This repository includes a comprehensive technical document [`Motion Profiling.pdf`](Motion%20Profiling.pdf) that provides:

- **Theoretical foundations** of motion profiling algorithms
- **Mathematical derivations** for trajectory generation
- **Step-by-step implementation guidance** for building your own motion profiling system
- **VEX-specific considerations** and optimization strategies
- **Visual examples** and diagrams explaining key concepts

This document is generated based on the typst document in `typst/`, any contributions are welcome.

I am also availible on discord at @comodomo, and am happy to respond to any queries about motion profiling.

## Contributing

This project was developed for VEX Robotics applications and is open to contributions from the VEX community:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.