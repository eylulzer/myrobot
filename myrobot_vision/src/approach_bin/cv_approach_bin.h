//
// Created by ros on 12.05.2020.
//

#ifndef ROS_WORKSPACE_CV_APPROACH_BIN_H
#define ROS_WORKSPACE_CV_APPROACH_BIN_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include "PointMeasure.h"
#include "FindCircles.h"
#include "CustomCircle.h"

class ApproachBin {
public:
    ApproachBin() : imageTransport(image_transport::ImageTransport(nodeHandle)) {}

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void detectCircles(const ros::TimerEvent &event);

    std::vector<std::vector<int>> groupByIntersection(const std::vector<CustomCircle>& mcArray);

    std::vector<CustomCircle> meanCircleArray(std::vector<CustomCircle> vector);

    void circleNavigation(const ros::TimerEvent &event);

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    image_transport::ImageTransport imageTransport;
    ros::Timer mainThread;
    ros::Timer navigationThread;
    image_transport::Subscriber raw_sub;
    image_transport::Subscriber rgb_sub;
    cv::Mat depthImage;
    cv::Mat rgbImage;
    PointMeasure pointMeasure;
    FindCircles findCircles;
    std::vector<CustomCircle> mCirclesArray;
};

#endif //ROS_WORKSPACE_CV_APPROACH_BIN_H
