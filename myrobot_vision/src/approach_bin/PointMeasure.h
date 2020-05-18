//
// Created by ros on 7.05.2020.
//

#ifndef ROS_WORKSPACE_POINTMEASURE_H
#define ROS_WORKSPACE_POINTMEASURE_H

#include <opencv2/imgproc/imgproc.hpp>
#include "CustomCircle.h"

// <1>
struct CameraInfo {
    double yAngle = 51.0;
    double xAngle = 91.0;
    double xPixels = 0;
    double yPixels = 0;
    cv::Point center = cv::Point(0.0, 0.0);
};

class PointMeasure {
public:
    static float getDepthFromPoint(const cv::Point &p, const cv::Mat &depthImage); // <2>
    double getAngleFromPoints(const cv::Point &a, const cv::Point &b) const; // <3>
    double getDistanceFromPoints(const cv::Point &a, const cv::Point &b, const cv::Mat &depthImage) const; // <4>
    double getDistanceFromCustomCircle(const CustomCircle &a, const CustomCircle &b) const; // <5>
    CameraInfo camera;
};


#endif //ROS_WORKSPACE_POINTMEASURE_H
