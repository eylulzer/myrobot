//
// Created by melodic on 03/06/2020.
//

#ifndef ROS_WORKSPACE_MYLINE_H
#define ROS_WORKSPACE_MYLINE_H

#include <opencv2/core/types.hpp>
#include <iostream>

class MyLine {
public:
    cv::Point2f start;
    cv::Point2f end;
    cv::Point2f middle;
    float slope;
    float length;

    MyLine(cv::Point2f _start, cv::Point2f _end);
    bool operator<(const MyLine &other) const;

    static bool areLinesEqual(const MyLine &thisLine, const MyLine &otherLine);

    static inline float lineDist(const MyLine &mLine);
    static  float slopeFactor(const MyLine &mLine);

    std::string toString() const;
};


#endif //ROS_WORKSPACE_MYLINE_H
