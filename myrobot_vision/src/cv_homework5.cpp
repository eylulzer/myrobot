//
// Created by ros on 27.04.2020.
//

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>

#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

bool requestMap(ros::NodeHandle &nh);

void readMap(const nav_msgs::OccupancyGrid &msg);

static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "load_map");
    ros::NodeHandle nodeHandle;

    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow("original Map", cv::WINDOW_NORMAL);

    if (!requestMap(nodeHandle))
        exit(-1);

    return 0;
}

bool requestMap(ros::NodeHandle &nh) {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
        ROS_INFO("Waiting for service static_map to become available");
    }

    ROS_INFO("Requesting the map...");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>(
        "static_map");

    if (mapClient.call(req, res)) {
        readMap(res.map);
        return true;
    } else {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

void readMap(const nav_msgs::OccupancyGrid &map) {
    int rows = map.info.width;
    int cols = map.info.height;
    ROS_INFO("Received a %d X %d map @ %.3f m/px\n", rows, cols, map.info.resolution);

    cv::Mat matMap(rows, cols, CV_8UC1);

    int currCell = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (map.data[currCell] == 0)
                matMap.at<uchar>(i, j, CV_8UC1) = 255; // unoccupied cell
            else if (map.data[currCell] == -1)
                matMap.at<uchar>(i, j, CV_8UC1) = 128;  //  unknown cell (-1)
            else
                matMap.at<uchar>(i, j, CV_8UC1) = 0; // occupied (1-100)

            currCell++;
        }
    }


    imshow("original Map", matMap);

    float scale = 4.00;
    cv::Mat blur, canny_output;
    cv::resize(matMap, blur, cv::Size(), 1.0/scale, 1.0/scale);
    cv::blur(blur, blur, cv::Size(3, 3));
    cv::Canny(blur, canny_output, 100, 100 * 2);

    vector<vector<cv::Point> > contours;
    cv::findContours(canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    vector<vector<cv::Point> > contours_poly(contours.size());
    vector<cv::Rect> boundRect(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = boundingRect(contours_poly[i]);
    }

    cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
    cv::Scalar color = cv::Scalar(0,255,0);
    cv::drawContours(drawing, contours_poly, 0, color);
    cv::rectangle(drawing, boundRect[0].tl(), boundRect[0].br(), color, 2);
    imshow("contours", drawing);

    cv::Point topLeft(boundRect[0].tl() * (scale - 0.1));
    cv::Point bottomRight(boundRect[0].br() *(scale+ 0.1));
    cv::Rect rect(topLeft, bottomRight);
    cv::Mat dst = matMap(rect);

    int erosion_type = cv::MORPH_ELLIPSE;
    cv::Mat element = getStructuringElement(erosion_type, cv::Size( 11, 11 ), cv::Point( 5, 5 ) );
    cv::erode( dst, dst, element );

    imshow(OPENCV_WINDOW, dst);
    cv::waitKey(0);
    cv::destroyWindow(OPENCV_WINDOW);
}


