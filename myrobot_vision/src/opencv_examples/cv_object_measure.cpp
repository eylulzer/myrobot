//
// Created by ros on 4.05.2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

float getDepthFromPoint(cv::Point &p);

double getAngleFromPoints(cv::Point &a, cv::Point &b);

double getDistanceFromPoints(cv::Point &a, cv::Point &b);

cv::Point3d get3dCoordinates(cv::Point &p2d);

static const std::string OPENCV_WINDOW = "Img Window";
ros::Publisher publisher;

struct CameraInfo {
    double xAngle = 95.0;
    double yAngle = 53.44;
    double xPixels = 1280;
    double yPixels = 720;
    cv::Point center = cv::Point(640, 360);
};

CameraInfo camera;
cv::Mat depthImage;


int main(int argc, char **argv) {
    ros::init(argc, argv, "object_distance");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera/rgb/image_raw", 1, imageCallback);

    image_transport::Subscriber rawimage_sub = imageTransport.subscribe(
            "/camera/depth/image_raw", 1, depthImageCallback);

    cv::namedWindow(OPENCV_WINDOW);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }
    // save on a global variable
    depthImage = cvImagePtr->image;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }

    //do operation over image
    cv::Point a, b;
    a = cv::Point(470, camera.center.y);
    b = cv::Point(775, camera.center.y);
    cv::circle(cvImagePtr->image, a, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(cvImagePtr->image, b, 2, cv::Scalar(0, 0, 255), -1);

    getDistanceFromPoints(a, b);

    std::cout << "\t-----" << std::endl;

    //Update GUI
    cv::imshow(OPENCV_WINDOW, cvImagePtr->image);
    cv::waitKey(3);
}


float getDepthFromPoint(cv::Point &p) {
    if (depthImage.empty())
        return -1.0;

    float depthInfo = 0.0;
    try {
        depthInfo = depthImage.at<float>(p);
    } catch (cv::Exception &e) {
        std::cout << "error : " << e.what() << std::endl;
    }

    std::cout << "Depth of " << p << " : " << depthInfo << std::endl;

    return depthInfo;
}

double getAngleFromPoints(cv::Point &a, cv::Point &b) {

    double x_distance = a.x - b.x;
    double y_distance = a.y - b.y;

    double x_angle = x_distance / camera.xPixels * camera.xAngle;
    double y_angle = y_distance / camera.yPixels * camera.yAngle;

    double angle = sqrt(pow(x_angle, 2) + pow(y_angle, 2));
    return angle;
}

double getDistanceFromPoints(cv::Point &a, cv::Point &b) {

    float a_d = getDepthFromPoint(a);
    float b_d = getDepthFromPoint(b);

    if (std::isnan(a_d) || std::isnan(b_d)) {
        return -1.0;
    }

    double angle = getAngleFromPoints(a, b) * M_PI / 180.0;

    // d2 = a*a + b*b - 2*a*b*cos(angle)
    double d = sqrt(pow(a_d, 2) + pow(b_d, 2) - 2 * a_d * b_d * cos(angle));
    std::cout << "distance " << " : " << d << "\tangle: " << angle << std::endl;

    return d;
}


cv::Point3d get3dCoordinates(cv::Point &p2d) {
    if (camera.center.x == 0.0 || depthImage.empty())
        return cv::Point3d(-10000, -10000, -10000);

    cv::Point3d p;

    double x = p2d.x - camera.center.x;
    double y = p2d.y - camera.center.y;
    float depth_diagonal = getDepthFromPoint(p2d);

    // find base diagonal of right angled square pyramid's
    double base_diagonal = sqrt(pow(x, 2) + pow(y, 2));

    std::cout << depth_diagonal << " : " << base_diagonal << std::endl;

    // find right angled square pyramid's height
    double z = sqrt(pow(depth_diagonal, 2) - pow(base_diagonal, 2));

    // convert to 3d point
    p = cv::Point3d(x, y, z);

    std::cout << "\n\n" << p << std::endl;

    return p;
}