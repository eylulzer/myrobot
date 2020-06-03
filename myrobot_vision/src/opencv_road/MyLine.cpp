//
// Created by melodic on 03/06/2020.
//

#include "MyLine.h"
#include <cmath>
#include <sstream>

MyLine::MyLine(cv::Point2f _start, cv::Point2f _end) : start{_start}, end{_end} {
    this->length = this->lineDist(*this);
    this->slope = this->slopeFactor(*this);
    this->middle = cv::Point((start.x + end.x) / 2.0, (start.y + end.y) / 2.0);
}


bool MyLine::operator<(const MyLine &other) const {
    return this->length < other.length;
}


bool MyLine::areLinesEqual(const MyLine &thisLine, const MyLine &other) {

    // middle point distance
    float dist = lineDist(MyLine(thisLine.middle, other.middle));

    if(thisLine.length == 0 || other.length == 0)
        return dist <= std::max(thisLine.length, other.length) * 0.5f;

    float product = ((thisLine.end.x - thisLine.start.x) * (other.end.x - other.start.x)) +
                    ((thisLine.end.y - thisLine.start.y) * (other.end.y - other.start.y));

    // dot product : 0 -> parallel , equal -> perpendicular
    if (fabs(product / (thisLine.length * other.length)) < cos(CV_PI / 30))
        return false;

    return dist <= std::max(thisLine.length, other.length) * 0.5f;

}

float MyLine::lineDist(const MyLine &mLine) {
    return sqrtf(
        pow((mLine.end.x - mLine.start.x), 2) +
        pow((mLine.end.y - mLine.start.y), 2)
    );
}

float MyLine::slopeFactor(const MyLine &myLine) {

    return ((myLine.end.x - myLine.start.x) == 0) ? 0 :
           ((myLine.end.y - myLine.start.y) /
           (myLine.end.x - myLine.start.x));
}

std::string MyLine::toString() const {
    std::ostringstream ss;
    ss << "start Point: " << this->start.x << " : " << this->start.y <<
       "\tend Point: " << this->end.x << " : " << this->end.y <<
       "\tlength: " << this->length <<
       "\tslope: " << this->slope;

    return ss.str();
}


