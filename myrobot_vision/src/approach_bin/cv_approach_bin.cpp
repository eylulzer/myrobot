//
// Created by ros on 7.05.2020.
//

#include <iostream>
#include <unordered_set>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "PointMeasure.h"
#include "FindCircles.h"
#include "CustomCircle.h"

class ApproachBin {
public:
    ApproachBin() : imageTransport(image_transport::ImageTransport(nodeHandle)) {}

    void init() {
        mainThread = nodeHandle.createTimer(ros::Duration(1.0), &ApproachBin::detectCircles, this);
        raw_sub = imageTransport.subscribe("/camera/rgb/image_raw", 1, &ApproachBin::imageCallback, this);
        rgb_sub = imageTransport.subscribe("/camera/depth/image_raw", 1, &ApproachBin::depthImageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
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

    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cvImagePtr;

        try {
            cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Error on CV_Bridge: %s", e.what());
        }
        // save on a global variable
        this->depthImage = cvImagePtr->image;
    }

    void detectCircles(const ros::TimerEvent &event) {
        mCirclesArray.clear();

        if (rgbImage.empty()) return;
        cv::Mat img = this->rgbImage.clone();

        for (const auto &item : this->findCircles.methodOne(this->rgbImage)){
//            cv::circle(img, cv::Point2f(item.x, item.y), item.radius, cv::Scalar(0,0,255), 2, cv::LINE_AA);
//            this->mCirclesArray.push_back(item);
//        }
//
//        for (const auto &item : this->findCircles.methodTwo(this->rgbImage)){
//            cv::circle(img, cv::Point2f(item.x, item.y), item.radius, cv::Scalar(0,255,0), 2, cv::LINE_AA);
//            this->mCirclesArray.push_back(item);
        }

        for (const auto &item : this->findCircles.methodThree(this->rgbImage)){
            cv::circle(img, cv::Point2f(item.x, item.y), item.radius, cv::Scalar(255,0,0), 2, cv::LINE_AA);
            this->mCirclesArray.push_back(item);
        }
//
//        for (const auto &item : this->findCircles.methodFour(this->rgbImage)){
//            cv::circle(img, cv::Point2f(item.x, item.y), item.radius, cv::Scalar(0,255,255), 2, cv::LINE_AA);
//            this->mCirclesArray.push_back(item);
//        }

        removeIntersection(mCirclesArray);

        for(const auto & c : mCirclesArray)
            circle(img, cv::Point2f(c.x, c.y), c.radius+100, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);

        cv::imshow("circles", img);
        cv::waitKey(3);

    }

    void removeIntersection(std::vector<CustomCircle> &circleList) {

        for (int i = 0; i < circleList.size(); i++) {
            for (int j = i+1; j < circleList.size(); j++){
                circleList[i].intersectsWith(circleList[j]);
            }
        }
    }


    void measureDistances() {

    }

private:
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    ros::Timer mainThread;
    image_transport::Subscriber raw_sub;
    image_transport::Subscriber rgb_sub;
    cv::Mat depthImage;
    cv::Mat rgbImage;
    PointMeasure pointMeasure;
    FindCircles findCircles;
    std::vector<CustomCircle> mCirclesArray;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_nav");
    ApproachBin approachBin;
    approachBin.init();

    ros::spin();
    return 0;
}