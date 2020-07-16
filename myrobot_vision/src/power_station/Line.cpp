//
// Created by melodic on 03/06/2020.
//

#include "Line.h"
#include <cmath>
#include <sstream>
#include <opencv2/core/types.hpp>

Line::Line(const cv::Point &_start, const cv::Point &_end) : start{_start}, end{_end} {
    this->length = Line::lineDist(*this);
    this->slope = Line::slopeFactor(*this);
}

int Line::lineDist(const Line &mLine) {
    return sqrt(
            pow((mLine.end.x - mLine.start.x), 2) +
            pow((mLine.end.y - mLine.start.y), 2)
    );
}

float Line::slopeFactor(const Line &myLine) {

    return ((myLine.end.x - myLine.start.x) == 0) ? 100.0 :
           (float (myLine.end.y - myLine.start.y) /
            float (myLine.end.x - myLine.start.x));
}

std::string Line::toString() const {
    std::ostringstream ss;
    ss << "start Point: " << this->start.x << " : " << this->start.y <<
       "\tend Point: " << this->end.x << " : " << this->end.y <<
       "\tlength: " << this->length <<
       "\tslope: " << this->slope;

    return ss.str();
}



