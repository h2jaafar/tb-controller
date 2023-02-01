#pragma once

#include <math.h>

namespace utils
{
struct Pose2
{
    double x;
    double y;
    double theta;
};

/**
* @brief Return the predefined path
* @param r radius 
* @param o_x origin x
* @param o_y origin y 
*/
std::vector<Pose2> X_ref(double r, double o_x, double o_y)
{
    std::vector<Pose2> X_ref_trajectory;

    int k = 16; // discretized steps 
    for(int i = 1; i <= k ; ++i)
    {
            Pose2 X; 
            double theta = i*360/k;
            X.x = r*cos(theta*M_PI/180) + o_x;
            X.y = r*sin(theta*M_PI/180) + o_y;
            X.theta = theta;
            X_ref_trajectory.push_back(X);
            std::cout << "i" << i << "x: " << X.x << " y: " << X.y << " th: " << X.theta << std::endl;
    }
    return X_ref_trajectory;
}
}