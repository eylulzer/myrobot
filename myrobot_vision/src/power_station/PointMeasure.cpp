//
// Created by ros on 7.05.2020.
//

#include "PointMeasure.h"
#include <iostream>

float PointMeasure::getDepthFromPoint(const cv::Point &p, const cv::Mat &depthImage) {
    if(depthImage.empty())
        return -1.0;

    float depthInfo = 0.0;
    try{
        depthInfo = depthImage.at<float>(p);
    } catch (cv::Exception &e) {
        std::cout << "error : " << e.what() << std::endl;
    }

    return depthInfo;
}

double PointMeasure::getAngleFromPoints(const cv::Point &a, const cv::Point &b) const {
    double x_distance = a.x - b.x;
    double y_distance = a.y - b.y;

    double x_angle = x_distance/this->camera.xPixels * this->camera.xAngle;
    double y_angle = y_distance/this->camera.yPixels * this->camera.yAngle;

    double angle = sqrt(pow(x_angle,2) + pow(y_angle, 2));
    return angle;
}

double PointMeasure::getDistanceFromPoints(const cv::Point &a, const cv::Point &b, const cv::Mat &depthImage) const {

    float a_d = this->getDepthFromPoint(a, depthImage);
    float b_d = this->getDepthFromPoint(b, depthImage);

    if (std::isnan(a_d) || std::isnan(b_d)) {
        return -1.0;
    }

    double angle = this->getAngleFromPoints(a, b) * M_PI / 180.0;

    // d2 = a*a + b*b - 2*a*b*cos(angle)
    double d = sqrt(pow(a_d, 2) + pow(b_d, 2) - 2.0 * a_d * b_d * cos(angle));

    return d;
}
