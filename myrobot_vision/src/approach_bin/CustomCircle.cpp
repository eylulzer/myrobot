//
// Created by ros on 11.05.2020.
//

#include "CustomCircle.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <unordered_set>

bool CustomCircle::operator==(const CustomCircle &other) {

    float t = 0.01;
    return (
        std::abs(this->x - other.x) < t &&
        std::abs(this->y - other.y) < t &&
        std::abs(this->radius - other.radius) < t
    );
}

// negative result for not intersecting, 0 for touch, positive for intersection
double CustomCircle::intersectsWith(const CustomCircle &other) {
    // distance between 2 points , in this case two circles' centers
    // distance = sqrt( (x2 - x1)2 + (y2 - y1)2 )
    double distance = sqrt(
        pow((other.x - this->x), 2) + pow((other.y - this->y), 2)
    );

    // sum of circles' radius
    double radiusSum = this->radius + other.radius;

    return (radiusSum - distance);
}
