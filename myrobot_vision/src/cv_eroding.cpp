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
int erosion_elem = 0;
int erosion_size = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_eroding");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera_link/image_raw", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOW);
    cv::createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", OPENCV_WINDOW,
                    &erosion_elem, 2);
    cv::createTrackbar( "Kernel size:\n 2n +1", OPENCV_WINDOW,
                    &erosion_size, 21);


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
    int erosion_type = 0;
    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

    cv::Mat dst;
    cv::Mat element = getStructuringElement( erosion_type,
                                         cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         cv::Point( erosion_size, erosion_size ) );
    erode( cvImagePtr->image, dst, element );

    //Update GUI
    cv::imshow(OPENCV_WINDOW, dst);
    cv::waitKey(3);
}
