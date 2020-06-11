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

bool MyLine::onSegment(cv::Point p, cv::Point q, cv::Point r) {
    return q.x <= fmax(p.x, r.x) && q.x >= fmin(p.x, r.x) &&
           q.y <= fmax(p.y, r.y) && q.y >= fmin(p.y, r.y);
}

bool MyLine::doLinesintersect(const MyLine &this_line, const MyLine &other_line) {

    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(this_line.start, this_line.end, other_line.start);
    int o2 = orientation(this_line.start, this_line.end, other_line.end);
    int o3 = orientation(other_line.start, other_line.end, this_line.start);
    int o4 = orientation(other_line.start, other_line.end, this_line.end);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // this_line.start, this_line.end and other_line.start are colinear and other_line.start lies on segment p1q1
    if (o1 == 0 && onSegment(this_line.start, other_line.start, this_line.end)) return true;

    // this_line.start, this_line.end and other_line.end are colinear and other_line.end lies on segment p1q1
    if (o2 == 0 && onSegment(this_line.start, other_line.end, this_line.end)) return true;

    // other_line.start, other_line.end and this_line.start are colinear and this_line.start lies on segment p2q2
    if (o3 == 0 && onSegment(other_line.start, this_line.start, other_line.end)) return true;

    // other_line.start, other_line.end and this_line.end are colinear and this_line.end lies on segment p2q2
    return o4 == 0 && onSegment(other_line.start, this_line.end, other_line.end);
}

int MyLine::orientation(cv::Point p, cv::Point q, cv::Point r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
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



