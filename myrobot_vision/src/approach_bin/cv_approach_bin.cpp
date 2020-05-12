//
// Created by ros on 7.05.2020.
//

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "CustomCircle.h"
#include "cv_approach_bin.h"


void ApproachBin::init() {
    mainThread = nodeHandle.createTimer(ros::Duration(0.2), &ApproachBin::detectCircles, this);
    navigationThread = nodeHandle.createTimer(ros::Duration(0.1), &ApproachBin::circleNavigation, this);
    raw_sub = imageTransport.subscribe("/camera/rgb/image_raw", 1, &ApproachBin::imageCallback, this);
    rgb_sub = imageTransport.subscribe("/camera/depth/image_raw", 1, &ApproachBin::depthImageCallback, this);
    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void ApproachBin::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (pointMeasure.camera.center.x == 0.0) {
            // configure camera info
            cv::Size sz = cvImagePtr->image.size();
            pointMeasure.camera.xPixels = sz.width;
            pointMeasure.camera.yPixels = sz.height;
            pointMeasure.camera.center = cv::Point(sz.width / 2, sz.height / 2);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }

    this->rgbImage = cvImagePtr->image;
}

void ApproachBin::depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }
    // save on a global variable
    this->depthImage = cvImagePtr->image;
}

void ApproachBin::detectCircles(const ros::TimerEvent &event) {
    mCirclesArray.clear();

    if (rgbImage.empty()) return;

    for (const auto &item : this->findCircles.methodOne(this->rgbImage))
        this->mCirclesArray.push_back(item);

    for (const auto &item : this->findCircles.methodTwo(this->rgbImage))
        this->mCirclesArray.push_back(item);

    for (const auto &item : this->findCircles.methodThree(this->rgbImage))
        this->mCirclesArray.push_back(item);

    for (const auto &item : this->findCircles.methodFour(this->rgbImage))
        this->mCirclesArray.push_back(item);

    mCirclesArray = this->meanCircleArray(mCirclesArray);
}

std::vector<std::vector<int>> ApproachBin::groupByIntersection(const std::vector<CustomCircle>& mcArray) {

    std::vector<std::vector<int>> intersections(mcArray.size());
    std::vector<int> processedIndex(mcArray.size(), -1);

    for (int i = 0; i < mcArray.size(); i++) {
        if (processedIndex[i] == -1)
            intersections[i].push_back(i);

        for (int j = i + 1; j < mcArray.size(); j++)
            if (processedIndex[i] == -1)
                if (mcArray[i].includesCircle(mcArray[j])) {
                    intersections[i].push_back(j);
                    processedIndex[j] = 0;
                }
    }

    return intersections;
}

std::vector<CustomCircle> ApproachBin::meanCircleArray(std::vector<CustomCircle> mcArray) {

    std::vector<CustomCircle> newList;
    for (const auto & v : this->groupByIntersection(mcArray)) {
        if(!v.empty()){
            CustomCircle total(0, 0, 0);
            for (const auto & cIndex : v)
            {
                total = CustomCircle(
                    mcArray[cIndex].x + total.x,
                    mcArray[cIndex].y + total.y,
                    mcArray[cIndex].radius + total.radius);
            }

            auto a = float(v.size());
            newList.emplace_back(total.x / a,total.y / a,total.radius /a);
        }
    }

    return newList;
}

void ApproachBin::circleNavigation(const ros::TimerEvent &event) {
    if(this->depthImage.empty() || this->rgbImage.empty())
        return;

    cv::Mat hsvImage;
    cv:cvtColor(this->rgbImage, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(18, 53, 9);
    cv::Scalar light_green(91, 159, 73);
    cv::inRange(hsvImage, dark_green, light_green, mask);

    //crop out top of image
    for (int y = 0; y < mask.rows - 2; y++) {
        if (y < 0 || y > mask.rows/2) {
            for (int x = 0; x < mask.cols; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

//  Use the moments() function to calculate the centroid of the blob of the binary image
    cv::Moments M = cv::moments(mask);
    cv::Mat img(this->rgbImage.size(), this->rgbImage.type());

    geometry_msgs::Twist cmd;
    cv::Point cntr(0, 0);

    if (M.m00 > 0) {
        cntr.x = int(M.m10 / M.m00);
        cntr.y = int(M.m01 / M.m00);

        cv::circle(img, cntr, 10, CV_RGB(255, 0, 0), -1);

        if (std::isnan(this->depthImage.at<float>(cntr)) || this->depthImage.at<float>(cntr) < 2.0)
            cmd.linear.x = 0.0;
        else
            cmd.linear.x = 0.2;

//      turn based on error
        cmd.angular.z = ((mask.cols / 2.0) - cntr.x) / 3000;
        if(std::abs(cmd.angular.z) < 0.05)
            cmd.angular.z = 0.0;

        publisher.publish(cmd);
    } else {
        //if no moments were created , perform rotatory recovery
        cmd.angular.z = 0.2;
        publisher.publish(cmd);
    }
}