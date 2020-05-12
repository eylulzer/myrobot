//
// Created by ros on 11.05.2020.
//

#ifndef ROS_WORKSPACE_CUSTOMCIRCLE_H
#define ROS_WORKSPACE_CUSTOMCIRCLE_H

#include <vector>
#include <iostream>

class CustomCircle {
public:

    float x;
    float y;
    float radius;

    CustomCircle() : x(0), y(0), radius(0) {}
    CustomCircle(float _x, float _y, float _r) : x{_x}, y{_y}, radius{_r} {}

    bool operator==(const CustomCircle &other) const;
    bool operator<(const CustomCircle &other) const;

//    double intersectsWith(const CustomCircle &other) const;
    bool includesCircle(const CustomCircle &other) const;
    std::string printCircle() const;
};

#endif //ROS_WORKSPACE_CUSTOMCIRCLE_H
