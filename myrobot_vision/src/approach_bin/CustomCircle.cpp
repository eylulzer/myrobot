//
// Created by ros on 11.05.2020.
//

#include "CustomCircle.h"
#include <cmath>
#include <sstream>

bool CustomCircle::operator==(const CustomCircle &other) const {

    float t = 0.01;
    return (
        std::abs(this->x - other.x) < t &&
        std::abs(this->y - other.y) < t &&
        std::abs(this->radius - other.radius) < t &&
        std::abs(this->range - other.range) < t
    );
}

bool CustomCircle::operator<(const CustomCircle &other) const {
    return this->radius < other.radius;
}

// returns true or false 
bool CustomCircle::includesCircle(const CustomCircle &other) const {
    float bigCircleRadius, smallCircleRadius;

    if (this->radius > other.radius){
        bigCircleRadius = this->radius;
        smallCircleRadius = other.radius;
    } else{
        bigCircleRadius = other.radius;
        smallCircleRadius = this->radius;
    }

    double distance = sqrt(
        pow((other.x - this->x), 2) + pow((other.y - this->y), 2)
    );

    // tolerance of 50%
    return (distance + smallCircleRadius <= bigCircleRadius + bigCircleRadius * 0.5);
}

bool CustomCircle::isInApproxWith(const CustomCircle &other, float pixelApprox, float distApprox) const {
    return (
        std::abs(this->x - other.x) < pixelApprox &&
        std::abs(this->y - other.y) < pixelApprox &&
        std::abs(this->radius - other.radius) < pixelApprox &&
        std::abs(this->range - other.range) < distApprox
    );
}

std::string CustomCircle::toString() const {
    std::ostringstream ss;
    ss << "x: " << this->x <<
        "\ty: " << this->y <<
        "\tradius: " << this->radius <<
        "\tdistance: " << this->range;

    return ss.str();
}

