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
    float range; // distance from camera;

    // <1>
    CustomCircle() : x{0}, y{0}, radius{0}, range{-1.0} {}
    CustomCircle(float _x, float _y, float _r) : x{_x}, y{_y}, radius{_r}, range{-1.0} {}
    CustomCircle(float _x, float _y, float _r, float _rng) : x{_x}, y{_y}, radius{_r}, range{_rng} {}

    //<2>
    bool operator==(const CustomCircle &other) const;
    bool operator<(const CustomCircle &other) const;

    bool includesCircle(const CustomCircle &other) const; //<3>
    bool isInApproxWith(const CustomCircle &other, float pixelApprox, float distApprox) const; //<4>
    std::string toString() const; //<5>
};

#endif //ROS_WORKSPACE_CUSTOMCIRCLE_H
