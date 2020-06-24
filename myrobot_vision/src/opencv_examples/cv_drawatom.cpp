//
// Created by ros on 20.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void MyEllipse(cv::Mat& img, double angle);
void MyFilledCircle(cv::Mat& img, cv::Point center);

static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_drawatom");
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
        ROS_ERROR("Error on CV_bridge : %s", e.what());
        return;
    }

    // do operation over the pic here
    cv::Mat atom_image = cvImagePtr->image;
    /// Creating ellipses
    MyEllipse(atom_image, 90);
    MyEllipse(atom_image, 0);
    MyEllipse(atom_image, 45);
    MyEllipse(atom_image, -45);

    // Creating circles
    MyFilledCircle(atom_image, cv::Point(atom_image.size().width / 2.0, atom_image.size().width / 2.0));

    //Update GUI Window
    cv::imshow(OPENCV_WINDOW, atom_image);
    cv::waitKey(3);
}

void MyEllipse(cv::Mat &img, double angle) {
    int thickness = 2;
    int lineType = 8;

    ellipse(img,
            cv::Point(img.size().width / 2.0, img.size().width / 2.0),
            cv::Size(img.size().width / 4.0, img.size().width / 16.0),
            angle,
            0,
            360,
            cv::Scalar(255, 0, 0), // blue
            thickness,
            lineType);
}

void MyFilledCircle( cv::Mat& img, cv::Point center )
{
    int thickness = -1;
    int lineType = 8;

    circle( img,
            center,
            img.size().width/32.0,
            cv::Scalar( 0, 0, 255 ),
            thickness,
            lineType );
}