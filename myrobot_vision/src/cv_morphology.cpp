//
// Created by ros on 29.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void Morphology_Operations(int, void *);

cv::Mat src, dst;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;
static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_morph");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);


    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL); // Create window
    cv::createTrackbar("Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat", OPENCV_WINDOW,
                       &morph_operator, max_operator);
    cv::createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", OPENCV_WINDOW,
                       &morph_elem, max_elem);
    cv::createTrackbar("Kernel size:\n 2n +1", OPENCV_WINDOW,
                       &morph_size, max_kernel_size);

    cv::namedWindow(OPENCV_WINDOW);
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
    src = cvImagePtr->image;

    // Since MORPH_X : 2,3,4,5 and 6
    int operation = morph_operator + 2;
    cv::Mat element = getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(src, dst, operation, element);

    //Update GUI
    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);
}
