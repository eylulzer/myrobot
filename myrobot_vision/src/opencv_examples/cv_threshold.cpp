//
// Created by ros on 30.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

int threshold_value = 0;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_threshold");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);


    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
    cv::createTrackbar( "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted",
                    OPENCV_WINDOW, &threshold_type,
                    max_type ); // Create Trackbar to choose type of Threshold
    cv::createTrackbar( "Value",
                    OPENCV_WINDOW, &threshold_value,
                    max_value ); // Create Trackbar to choose Threshold value
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    //do operation over image
    cv::Mat src_gray, dst;
    cv::Mat src = cvImagePtr->image;
    cvtColor(src, src_gray, cv::COLOR_BGR2GRAY ); // Convert the image to Gray
    threshold(src_gray, dst, threshold_value, max_BINARY_value,threshold_type );

    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);
}