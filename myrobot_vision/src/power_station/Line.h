//
// Created by melodic on 03/06/2020.
//

#ifndef ROS_WORKSPACE_LINE_H
#define ROS_WORKSPACE_LINE_H

#include <iostream>
#include <opencv2/core/types.hpp>

class Line {
public:
    cv::Point start;
    cv::Point end;
    float slope;
    float length;

    Line(const cv::Point &_start, const cv::Point &_end);

    static inline int lineDist(const Line &mLine);

    static inline float slopeFactor(const Line &mLine);

    std::string toString() const;
};


#endif //ROS_WORKSPACE_LINE_H
