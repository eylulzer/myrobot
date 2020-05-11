//
// Created by ros on 11.05.2020.
//

#ifndef ROS_WORKSPACE_CUSTOMCIRCLE_H
#define ROS_WORKSPACE_CUSTOMCIRCLE_H

#include <vector>

class CustomCircle {
public:
    CustomCircle(float _x, float _y, float _r)
    : x{_x}, y{_y}, radius{_r} {}

    float x;
    float y;
    float radius;

    bool operator== (const CustomCircle &other);

    double intersectsWith(const CustomCircle &other);
};

#endif //ROS_WORKSPACE_CUSTOMCIRCLE_H
