//
// Created by ros on 7.05.2020.
//

#ifndef ROS_WORKSPACE_POINTMEASURE_H
#define ROS_WORKSPACE_POINTMEASURE_H

#include <opencv2/imgproc/imgproc.hpp>

struct CameraInfo {
    double yAngle = 51.0;
    double xAngle = 91.0;
    double xPixels = 0;
    double yPixels = 0;
    cv::Point center = cv::Point(0.0, 0.0);
};

class PointMeasure {
public:
    float getDepthFromPoint(const cv::Point &p, const cv::Mat &depthImage);
    double getAngleFromPoints(const cv::Point &a, const cv::Point &b, const cv::Mat &depthImage);
    double getDistanceFromPoints(const cv::Point &a, const cv::Point &b, const cv::Mat &depthImage);
    CameraInfo camera;
};


#endif //ROS_WORKSPACE_POINTMEASURE_H
