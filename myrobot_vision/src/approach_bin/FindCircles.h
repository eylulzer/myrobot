//
// Created by ros on 7.05.2020.
//

#ifndef ROS_WORKSPACE_FINDCIRCLES_H
#define ROS_WORKSPACE_FINDCIRCLES_H

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "CustomCircle.h"

class FindCircles {
public:
    std::vector<CustomCircle> methodOne(const cv::Mat &image);

    std::vector<CustomCircle> methodTwo(const cv::Mat &image);

    std::vector<CustomCircle> methodThree(const cv::Mat &image);

    std::vector<CustomCircle> methodFour(const cv::Mat &image);
};


#endif //ROS_WORKSPACE_FINDCIRCLES_H
