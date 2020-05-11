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
double  getDistanceFromPoints(cv::Point &a, cv::Point &b);

static const std::string OPENCV_WINDOW = "1: Crop, Mask, Canny Edges, Hough Circles";
ros::Publisher publisher;

struct CameraInfo {
    double yAngle = 51.0;
    double xAngle = 91.0;
    double xPixels = 0;
    double yPixels = 0;
    cv::Point center = cv::Point(0.0, 0.0);
};

CameraInfo camera;
cv::Mat depthImage;


int main(int argc, char **argv) {
    ros::init(argc, argv, "binwheels_canny");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);

    image_transport::Subscriber rawimage_sub = imageTransport.subscribe(
        "/camera/depth/image_raw", 1, depthImageCallback);

    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr &msg){
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
        if (camera.center.x == 0.0) {
            // configure camera info
            cv::Size sz = cvImagePtr->image.size();
            camera.xPixels = sz.width;
            camera.yPixels = sz.height;
            camera.center = cv::Point(sz.width/2, sz.height/2);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }

    cv::Mat image = cvImagePtr->image;

    //do operation over image
    cv::Mat hsvImage, dst, gray;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // closing
    int morph_size = 20;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(hsvImage, hsvImage, 3, element);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(10, 50, 10);
    cv::Scalar light_green(80, 255, 80);
    cv::inRange(hsvImage, dark_green, light_green, mask);

    // put black over hsvImage bin mask
    cv::Mat black = cv::Mat::zeros(hsvImage.size(), hsvImage.type());
    black.copyTo(hsvImage, mask);

    //imshow("hsvImage after mask", hsvImage);

    // bottom part of image
    cv::Point cropOrigin(0, image.rows/2 - 20);
    cv::Size s(image.cols, image.rows - (image.rows/2 - 20));
    cv::Mat im = hsvImage(cv::Rect(cropOrigin, s));
    im = im.clone();

    //blur
    cv::medianBlur(im, dst,5);


    //sharpen
    cv::Mat kern = (cv::Mat_<char>(3, 3) << 0, -1, 0,
        -1, 5, -1,
        0, -1, 0);

    cv::filter2D(dst, dst, dst.depth(), kern);



    //add brightness
    cv::Mat toCanny, edges;
    dst.convertTo(toCanny, -1, 2, 10);

    //canny edge detector
    int lowThreshold = 90;
    Canny( toCanny, edges, lowThreshold, lowThreshold*3, 3);
    dst = cv::Scalar::all(0);
    im.copyTo( dst, edges);


    imshow("edge", dst);


    // black n white
    cvtColor(dst, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     100,  // change this value to detect circles with different distances to each other
                     100, 31, 10, 100 // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
    );

    for (auto & i : circles) {
        cv::Vec3i c = i;
        cv::Point center = cv::Point(c[0], c[1] + (image.rows/2 - 20));
        // circle outline
        int radius = c[2];
        circle(image, center, radius, cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
    }


    //Update GUI
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(3);
}


float getDepthFromPoint(cv::Point &p){
    if(depthImage.empty())
        return -1.0;

    float depthInfo = 0.0;
    try{
        depthInfo = depthImage.at<float>(p);
    } catch (cv_bridge::Exception &e) {
        std::cout << "error : " << e.what() << std::endl;
    }
    return depthInfo;
}

double getAngleFromPoints(cv::Point &a, cv::Point &b){

    double x_distance = a.x - b.x;
    double y_distance = a.y - b.y;

    double x_angle = x_distance/camera.xPixels * camera.xAngle;
    double y_angle = y_distance/camera.yPixels * camera.yAngle;

    double angle = sqrt(pow(x_angle,2) + pow(y_angle, 2));

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

    return d;
}