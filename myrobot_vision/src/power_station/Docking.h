//
// Created by melodic on 7/15/20.
//

#ifndef ROS_WORKSPACE_DOCKING_H
#define ROS_WORKSPACE_DOCKING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "PointMeasure.h"
#include "Line.h"

typedef struct {
    std::string type;
    std::string data;
    std::vector<cv::Point> location;
} decodedObject;

struct AmclGoal {
    double x = -3.32913665192;
    double y = 2.97783347511;
    double z = -0.711961822796; // orientation
    double w = 0.702218173277;

    double distanceToQR = 0.15; //m
    std::string qrData = "Power Station";
    std::string type = "QR-Code";
    int qrVerticalTolerance = 3; //px
};

class Docking {
public:
    Docking() : imageTransport{image_transport::ImageTransport(this->nodeHandle)},
                goalReached{false},
                correctZ{false},
                correctX{false} {};

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void depthCallback(const sensor_msgs::ImageConstPtr &msg);

    void amclPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void decode(const ros::TimerEvent &event);

    void moveQR(const ros::TimerEvent &event);

    void correctAngular(const float &dist, const cv::Point &center, const float &verticalDiff);

    void correctLinear(const float &dist, const cv::Point &center);

private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber amclSub;
    ros::Publisher cmdPub;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSub;
    image_transport::Subscriber depthSub;

    cv::Mat rgbImage;
    cv::Mat depthImage;
    geometry_msgs::PoseWithCovarianceStampedConstPtr robotPosition;

    std::string windowName;
    std::vector<decodedObject> decodedObjects;

    ros::Timer decodeThread;
    ros::Timer navThread;

    PointMeasure pointMeasure;
    AmclGoal powerGoal;

    bool goalReached, correctZ, correctX;
    std::vector<Line> verticalLines;
};


#endif //ROS_WORKSPACE_DOCKING_H
