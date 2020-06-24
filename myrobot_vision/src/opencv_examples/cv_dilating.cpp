//
// Created by ros on 21.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Img Window";
int dilation_elem = 0;
int dilation_size = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_dilating");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera/rgb/image_raw", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOW);
    cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", OPENCV_WINDOW,
                       &dilation_elem, 2);
    cv::createTrackbar( "Kernel size:\n 2n +1", OPENCV_WINDOW,
                        &dilation_size, 21);

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
    int dilation_type = 0;
    if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }

    cv::Mat dst;
    cv::Mat element = getStructuringElement( dilation_type,
                                         cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         cv::Point( dilation_size, dilation_size ) );

    dilate( cvImagePtr->image, dst, element );

    //Update GUI
    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);
}
