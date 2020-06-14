//
// Created by melodic on 03/06/2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <geometry_msgs/Twist.h>
#include "MyLine.h"

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

cv::Mat getROI(const cv::Mat &mRoi, cv::Point roi_points[], int n);

std::vector<MyLine> groupLines(std::vector<cv::Vec4i> &mLines);

ros::Publisher pub;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_eroding");
    ros::NodeHandle nodeHandle;
    pub = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);

    cv::namedWindow("OPENCV_WINDOW");
    ros::spin();
    cv::destroyAllWindows();

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    geometry_msgs::Twist cmd;
    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    cv::Mat original = cvImagePtr->image.clone();

    cv::Point origin = cv::Point(0, original.size().height / 2);
    cv::Point size = cv::Point(original.size().width, original.size().height);
    cv::Rect r = cv::Rect(origin, size);
    cv::Mat mRoi = original(r);

    //add contrast
    mRoi.convertTo(mRoi, -1, 2, 10);

    int erosion_size = 9;
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                            cv::Point(erosion_size, erosion_size));


    //blur, erosion, blur, sharpen; make patterns dissapear, leave only edges
    cv::blur(mRoi, mRoi, cv::Size(20, 20));
    cv::erode(mRoi, mRoi, element);
    cv::medianBlur(mRoi, mRoi, 35);

    cv::Mat mask;

    cvtColor(mRoi, mRoi, cv::COLOR_BGR2HSV);
    inRange(mRoi, cv::Scalar::all(0), cv::Scalar(1, 1, 70), mask);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_poly(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
    }

    cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(drawing, contours_poly, -1, cv::Scalar::all(255), 5);

    cv::Point roi_pts[4];

    roi_pts[0] = cv::Point(drawing.size().width * 0.10, drawing.size().height * 0.30); // tl
    roi_pts[1] = cv::Point(drawing.size().width * 0.05, drawing.size().height * 0.80); // bl
    roi_pts[2] = cv::Point(drawing.size().width * 0.95, drawing.size().height * 0.80); // br
    roi_pts[3] = cv::Point(drawing.size().width * 0.90, drawing.size().height * 0.30); // tr

    mRoi = getROI(drawing, roi_pts, 4);

    cv::Mat linedst;
    std::vector<cv::Vec4i> lines;
    HoughLinesP(mRoi, lines, 1, CV_PI / 180, 50, 100, 10);

    std::vector<MyLine> finalLines = groupLines(lines);

    switch (finalLines.size()) {
        case 0:
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
        case 1:
            cmd.linear.x = 0.2;
            cmd.angular.z =  (finalLines[0].slope > 0) ? 0.2 : -0.2;
            break;
        default:
            break;
    }

    if (finalLines.size() >= 2) {

        bool plus_side_flag = false;
        bool minus_side_flag = false;
        bool lines_intersect = false;
        int to = 200; //px
        int center_width = mRoi.size().width / 2;

        for (int i = 0; i < finalLines.size() ; i++) {
            if (center_width + to > finalLines[i].start.x || center_width + to  > finalLines[i].end.x)
                minus_side_flag = true;

            if (center_width - to < finalLines[i].start.x || center_width - to < finalLines[i].end.x)
                plus_side_flag = true;

            if(finalLines.size() - 1 > i)
                if(MyLine::doLinesintersect(finalLines[i], finalLines[i+1]))
                    lines_intersect = true;
        }

        cmd.linear.x = 0.05;
        if (!plus_side_flag || !minus_side_flag) // means that all lines are on the same side
            cmd.angular.z = (!plus_side_flag) ? 0.1 : -0.1;
        else if (lines_intersect && finalLines.size() == 2){
            cmd.angular.z = fmin(finalLines[0].middle.x, finalLines[1].middle.x) > center_width ? -0.15 : 0.15;
        } else {
            cv::Mat white(mRoi.size(), CV_8UC1, cv::Scalar::all(255));
            int n = finalLines.size() * 2;
            cv::Point rp[n];

            for (int i = 0; i < n; i += 2) {
                rp[i] = finalLines[i / 2].start;
                rp[i + 1] = finalLines[i / 2].end;
            }

            cv::Mat tRoi = getROI(white, rp, n);

            cv::Moments M = cv::moments(tRoi);
            cv::Point tRoi_center(0, 0);

            if (M.m00 > 0) {
                tRoi_center.x = int(M.m10 / M.m00);
                tRoi_center.y = int(M.m01 / M.m00);

                cv::circle(cvImagePtr->image, tRoi_center, 50, cv::Scalar(0,255, 100), -1);

                float err = (tRoi.size().width / 2 - tRoi_center.x) / 1000.0;
                cmd.angular.z = (std::abs(err) < 0.075) ? 0.0 : err;
            }
        }
    }

    pub.publish(cmd);

    // draw roi
    for (const auto &l : finalLines) {
        line(
            cvImagePtr->image,
            cv::Point(l.start.x, l.start.y + original.size().height / 2),
            cv::Point(l.end.x, l.end.y + original.size().height / 2),
            cv::Scalar(0, 255, 255),
            5,
            cv::LINE_AA
        );
    }

    cv::drawContours(cvImagePtr->image,
                     contours_poly,
                     -1,
                     cv::Scalar(0, 0, 255),
                     3,
                     cv::LINE_8,
                     cv::_InputArray(),
                     0x7fffffff,
                     cv::Point(0, original.size().height / 2)
    );

    cv::resize(cvImagePtr->image, cvImagePtr->image, cv::Size(), 0.5, 0.5);
    cv::imshow("OPENCV_WINDOW", cvImagePtr->image);
    cv::waitKey(3);
}


cv::Mat getROI(const cv::Mat &mRoi, cv::Point roi_points[], int n) {

    cv::Mat mask = cv::Mat::zeros(mRoi.size(), mRoi.type());
    cv::Mat black = cv::Mat::zeros(mRoi.size(), mRoi.type());

    const cv::Point *ppt[1] = {roi_points};
    int npt[] = {n};
    cv::fillPoly(black, ppt, npt, 1, cv::Scalar::all(255));

    mRoi.copyTo(mask, black);
    return mask;
}


std::vector<MyLine> groupLines(std::vector<cv::Vec4i> &mLines) {

    std::vector<MyLine> mLinesVector;
    for (const cv::Vec4i &v : mLines)
        mLinesVector.emplace_back(cv::Point2f(v[0], v[1]), cv::Point2f(v[2], v[3]));

    std::vector<int> labels;
    int numberOfLines = cv::partition(mLinesVector, labels, MyLine::areLinesEqual);

    std::vector<MyLine> finalLines;

    for(const auto &l : labels)
        std::cout << l << "\t";
    std::cout<<std::endl<<std::endl;

    for (int i = 0; i < numberOfLines; i++) {
        std::vector<float> coords_x, coords_y;

        float mSlope;
        for (int j = 0; j < labels.size(); j++) {
            if (labels[j] == i) {
                coords_x.push_back(mLinesVector[j].start.x);
                coords_x.push_back(mLinesVector[j].end.x);
                coords_y.push_back(mLinesVector[j].start.y);
                coords_y.push_back(mLinesVector[j].end.y);
                mSlope = mLinesVector[j].slope;
            }
        }

        auto result_x = std::minmax_element(coords_x.begin(), coords_x.end());
        auto result_y = std::minmax_element(coords_y.begin(), coords_y.end());


        if (mSlope > 0) {
            finalLines.emplace_back(
                cv::Point2f(*result_x.first, *result_y.first),   // minumum, minimum
                cv::Point2f(*result_x.second, *result_y.second)); // maximum, maximum
        } else
            finalLines.emplace_back(
                cv::Point2f(*result_x.first, *result_y.second), // minumum, maximum
                cv::Point2f(*result_x.second, *result_y.first)); // maximum, minumum

    }

    return finalLines;
}

