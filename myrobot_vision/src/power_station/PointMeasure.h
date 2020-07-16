//
// Created by ros on 7.05.2020.
//

#ifndef ROS_WORKSPACE_POINTMEASURE_H
#define ROS_WORKSPACE_POINTMEASURE_H

#include <opencv2/imgproc/imgproc.hpp>

struct CameraInfo {
    double xAngle = 95.0;
    double yAngle = 53.44;
    double xPixels = 1280;
    double yPixels = 720;
    cv::Point center = cv::Point(640, 360);
};

class PointMeasure {
public:
    static float getDepthFromPoint(const cv::Point &p, const cv::Mat &depthImage);
    double getAngleFromPoints(const cv::Point &a, const cv::Point &b) const;
    double getDistanceFromPoints(const cv::Point &a, const cv::Point &b, const cv::Mat &depthImage) const;

    CameraInfo camera;
};

#endif //ROS_WORKSPACE_POINTMEASURE_H
