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

    void init(); // <1>

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void detectObstacle(const ros::TimerEvent &event); // <2>

    void detectCircles(const ros::TimerEvent &event); // <3>

    std::vector<CustomCircle> meanCircleArray(std::vector<CustomCircle> vector); // <4>

    std::vector<std::vector<int>> groupByIntersection(const std::vector<CustomCircle>& mcArray); // <5>

    void removeDuplicatesByDepth(std::vector<CustomCircle> &ccArray); // <6>

    void circleNavigation(const ros::TimerEvent &event); // <7>

    void getUnderBin(geometry_msgs::Twist & cmd); // <8>

    void closeNav(const ros::TimerEvent &event); // <9>

    cv::Mat maskGreens(); // <10>

    double imageMatched(); // <11>

    bool isDone;


private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber raw_sub;
    image_transport::Subscriber rgb_sub;

    ros::Timer obstacleThread;
    ros::Timer mainThread;
    ros::Timer navigationThread;
    ros::Timer closeNavThread;

    cv::Mat depthImage;
    cv::Mat rgbImage;
    PointMeasure pointMeasure;
    FindCircles findCircles;
    std::vector<CustomCircle> mCirclesArray;
    std::vector<cv::Mat> templArray;
    CustomCircle goal;
    std::vector<bool> canGo; // 0 left, 1 center, 2 right
};

#endif //ROS_WORKSPACE_CV_APPROACH_BIN_H
