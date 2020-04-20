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

double contrast;

static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_contrast");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera_link/image_raw", 1, imageCallback);

    contrast = 1.0; // 1.0-3.0 value

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
    cv::Mat new_image = cv::Mat::zeros(cvImagePtr->image.size(), cvImagePtr->image.type());

    /// Do the operation new_image(i,j) = alpha*image(i,j) + beta
    for (int y = 0; y < cvImagePtr->image.rows; y++) {
        for (int x = 0; x < cvImagePtr->image.cols; x++) {
            for (int c = 0; c < 3; c++) {
                new_image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                        contrast * (cvImagePtr->image.at<cv::Vec3b>(y, x)[c]));
            }
        }
    }

    //Update GUI
    cv::imshow(OPENCV_WINDOW, new_image);
    cv::waitKey(3);
}