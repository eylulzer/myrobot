//
// Created by ros on 5.05.2020.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>


void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);
float getDepthFromPoint(cv::Point &p);
double getAngleFromPoints(cv::Point &a, cv::Point &b);
double  getDistanceFromPoints(cv::Point &a, cv::Point &b);

static const std::string OPENCV_WINDOW = "4 : Closing, Blob ";
ros::Publisher publisher;

struct CameraInfo {
    double yAngle = 52.5;
    double xAngle = 73.0;
    double xPixels = 0;
    double yPixels = 0;
    cv::Point center = cv::Point(0.0, 0.0);
};

CameraInfo camera;
cv::Mat depthImage;


int main(int argc, char **argv) {
    ros::init(argc, argv, "binwheels_blobs");
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
    cv::Mat im;

    // closing
    int morph_size = 15;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(image, im, 3, element);

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.05;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.2;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    // Storage for blobs
    std::vector<cv::KeyPoint> keypoints;


    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect(im, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
    // the size of the circle corresponds to the size of blob

    cv::Mat im_with_keypoints;
    drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Show blobs
    imshow(OPENCV_WINDOW, im_with_keypoints);
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

    // d = a*a + b*b - 2*a*b*cos(angle)
    double d = pow(a_d, 2) + pow(b_d, 2) - 2 * a_d * b_d * cos(angle);

    return d;
}