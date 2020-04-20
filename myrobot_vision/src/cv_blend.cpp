//
// Created by ros on 16.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

static const std::string OPENCV_WINDOW = "Img Window";
//cv::Mat src;

int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv_blend");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera_link/image_raw", 1, imageCallback);

    //Since we are adding camera image and src, they both have to be of the same size (width and height) and type.
//    src = cv::imread("/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/index.jpg");
//    if( !src.data ) { printf("Error loading src image \n"); return -1; }

    cv::namedWindow(OPENCV_WINDOW);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    //do operation over image
//    double alpha = 0.9;
//    double beta = ( 1.0 - alpha );
//    cv::Mat destination;
//    addWeighted( cvImagePtr->image, alpha, src, beta, 0.0, destination);

    cv::Mat new_image = cv::Mat::zeros(cvImagePtr->image.size(), cvImagePtr->image.type()); //black image
    cv::Mat destination;
    addWeighted( cvImagePtr->image, 0.5, new_image, 0.5, 0.0, destination); //overlay black image

    //Update GUI
    cv::imshow(OPENCV_WINDOW, destination);
    cv::waitKey(3);
}