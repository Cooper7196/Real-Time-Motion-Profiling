#pragma once

class Pose
{
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