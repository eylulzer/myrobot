//
// Created by ros on 20.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Img Window";

int height;
int width;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_crop");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera_link/image_raw", 1, imageCallback);

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
    height = cvImagePtr->image.size().height;
    width = cvImagePtr->image.size().width;

    cv::Point origin(0, 0);
    cv::Point size(width - 400 , height - 400);
    cv::Rect r(origin, size);
    cv::Mat smallImg = cvImagePtr->image(r);


    //Update GUI
    cv::imshow(OPENCV_WINDOW, smallImg);
    cv::waitKey(3);
}