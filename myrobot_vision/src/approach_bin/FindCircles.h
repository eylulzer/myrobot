//
// Created by ros on 7.05.2020.
//

#ifndef ROS_WORKSPACE_FINDCIRCLES_H
#define ROS_WORKSPACE_FINDCIRCLES_H

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

class FindCircles {
public:
    std::vector<cv::Vec3f> methodOne(const cv::Mat &image);

    std::vector<cv::Vec3f> methodTwo(const cv::Mat &image);

    std::vector<cv::Vec3f> methodThree(const cv::Mat &image);

    std::vector<cv::Vec3f> methodFour(const cv::Mat &image);
};


#endif //ROS_WORKSPACE_FINDCIRCLES_H
