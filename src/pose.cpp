#include "pose.hpp"
#include <math.h>

double Pose::distance(Pose pose)
{
    return sqrt(pow(pose.x - this->x, 2) + pow(pose.y - this->y, 2));
}
double Pose::angle(Pose pose)
{
    return atan2(pose.y - this->y, pose.x - this->x);
}

// std::ostream &operator<<(std::ostream &strm, const Pose &a)
// {
//     return strm << "(" << a.x << "," << a.y << "," << radToDeg(a.theta) << ")";
// }
