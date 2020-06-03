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

cv::Mat getROI(const cv::Mat &mRoi);

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
    std::vector<cv::Rect> boundRect(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
    }

    cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(drawing, contours_poly, -1, cv::Scalar::all(255), 5);

    mRoi = getROI(drawing);

    cv::Mat linedst;
    std::vector<cv::Vec4i> lines;
    HoughLinesP(mRoi, lines, 1, CV_PI / 180, 50, 100, 10);

    std::vector<MyLine> finalLines = groupLines(lines);

    // draw roi
    for (const auto &l : finalLines) {
        line(
            cvImagePtr->image,
            cv::Point(l.start.x, l.start.y + original.size().height / 2),
            cv::Point(l.end.x, l.end.y + original.size().height / 2),
            cv::Scalar(0, 255, 255),
            5,
            CV_AA
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

    cv::resize(cvImagePtr->image, cvImagePtr->image, cv::Size(), 0.75, 0.75);
    cv::imshow("OPENCV_WINDOW", cvImagePtr->image);
    cv::waitKey(3);
}


cv::Mat getROI(const cv::Mat &mRoi) {
    cv::Point bottom_left(mRoi.size().width * 0.05, mRoi.size().height * 0.80);
    cv::Point top_left(mRoi.size().width * 0.10, mRoi.size().height * 0.30);
    cv::Point bottom_right(mRoi.size().width * 0.95, mRoi.size().height * 0.80);
    cv::Point top_right(mRoi.size().width * 0.90, mRoi.size().height * 0.30);

    cv::Mat mask = cv::Mat::zeros(mRoi.size(), mRoi.type());
    cv::Mat black = cv::Mat::zeros(mRoi.size(), mRoi.type());
    cv::Point roi_points[4];
    roi_points[0] = top_left;
    roi_points[1] = bottom_left;
    roi_points[2] = bottom_right;
    roi_points[3] = top_right;

    const cv::Point *ppt[1] = {roi_points};
    int npt[] = {4};
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
                cv::Point2f(*result_x.first, *result_y.first),
                cv::Point2f(*result_x.second, *result_y.second));
        } else
            finalLines.emplace_back(
                cv::Point2f(*result_x.first, *result_y.second),
                cv::Point2f(*result_x.second, *result_y.first));

    }

    return finalLines;
}
