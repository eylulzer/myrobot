//
// Created by ros on 7.05.2020.
//

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "PointMeasure.h"
#include "FindCircles.h"

class ApproachBin {
public:
    ApproachBin()
    :
    imageTransport(image_transport::ImageTransport(nodeHandle)),
    mainThread(nodeHandle.createTimer(ros::Duration(5.0), &ApproachBin::detectCircles, this, false))
    {
        imageTransport.subscribe("/camera/rgb/image_raw", 1, &ApproachBin::imageCallback, this);
        imageTransport.subscribe("/camera/depth/image_raw", 1, &ApproachBin::depthImageCallback, this);
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

        cv::Mat image = cvImagePtr->image;
        imshow("image", image);
        cv::waitKey(3);
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
        mCirclesArray.push_back(this->findCircles.methodOne(  this->rgbImage));
        mCirclesArray.push_back(this->findCircles.methodTwo(  this->rgbImage));
        mCirclesArray.push_back(this->findCircles.methodThree(this->rgbImage));
        mCirclesArray.push_back(this->findCircles.methodFour( this->rgbImage));

        for(const auto &item: mCirclesArray)
            std::cout << item.size() << "\t";

        std::cout<<std::endl;
    }

private:
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    ros::Timer mainThread;
    cv::Mat depthImage;
    cv::Mat rgbImage;
    PointMeasure pointMeasure;
    FindCircles findCircles;
    std::vector<std::vector<cv::Vec3f>> mCirclesArray;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_nav");
    ApproachBin approachBin;

    ros::spin();
    return 0;
}