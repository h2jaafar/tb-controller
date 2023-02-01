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


/**
 * @brief Computes the intersection points between two circles, 1 and 2
 * @param x1: x of circle 1
 * @param y1: y of circle 1
 * @param x2: x of circle 2
 * @param y1: y of circle 2
 * @param r1: r of circle 1
 * @param r2: 2 of circle 2
 * @return vector with two intersection points
 * 
*/
std::vector<Pose2> circle_intersection(double x1, double y1, double r1, double x2, double y2, double r2)
{
    // https://lucidar.me/en/mathematics/how-to-calculate-the-intersection-points-of-two-circles/#intersection-points
    double d = sqrt(pow((x2-x1),2) + pow((y2-y1),2));
    double a = (pow(r1,2) - pow(r2, 2) + pow(d,2)) / (2*d);
    // double b = (pow(r2,2) - pow(r2, 2) + pow(d,2)) / (2*d);
    double h = sqrt(pow(r1, 2) - pow(a,2));

    double x5 = x1 + (a/d)*(x2 - x1);
    double y5 = y1 + (a/d)*(y2 - y1);

    Pose2 c1;
    Pose2 c2;
    c1.x = x5 - (h*(y2 - y1)/d);
    c1.y = y5 + (h*(x2 - x1)/d);

    c2.x = x5 + (h*(y2 - y1)/d);
    c2.y = y5 - (h*(x2 - x1)/d);

    std::vector<Pose2> intersection_points;
    intersection_points.push_back(c1);
    intersection_points.push_back(c2);

    return intersection_points;
}

/**
 * @brief return the correct intersection point, assuming robot is moving ccw
 * @param c1 intersection 1
 * @param c2 intersection 2
 * @param x2 X Position of robot  
 * @param y2 y position of robot
 * @return Pose2 of the next intersection point
 * 
*/
Pose2 find_correct_intersection(Pose2 c1, Pose2 c2, double x2, double y2)
{
    double theta = atan2(y2, x2)* 180 / M_PI;
    if(theta <= 180)
    {
        // take the left most point, i.e lowest x
        if(c1.x < c2.x)
        {return c1;}
        else{ return c2;}
    }
    else
    {   // take right most point, i.e biggest x
        if(c1.x > c2.x)
        {return c1;}
        else{return c2;}
    }
}
}