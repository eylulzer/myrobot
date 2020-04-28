//
// Created by ros on 28.04.2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Img Window";
ros::Publisher publisher;

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_bin");
    ros::NodeHandle nodeHandle;
    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
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

    // Convert input image to HSV
    cv::Mat image = cvImagePtr->image;
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(20, 80, 20);
    cv::Scalar light_green(80, 255, 80);
    cv::inRange(hsvImage, dark_green, light_green, mask);

    //cropping out top half of image
    int width = mask.cols;
    int height = mask.rows;

    int search_top = 0;
    int search_bottom = search_top + height/2;

    // Zero out pixels outside the desired region
    for (int y = 0; y < height - 2; y++) {
        if (y < search_top || y > search_bottom) {
            for (int x = 0; x < width; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

//  Use the moments() function to calculate the centroid of the blob of the binary image
    cv::Moments M = cv::moments(mask);

    geometry_msgs::Twist cmd;

    if (M.m00 > 0) {
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);

//      Move the robot in proportion to the error signal
        int err = cx - width / 2;

//      go forward towards the center of the dumpster
//      if too close, dumpster green will be left in the first eighth of image, go back
//      if in between , stop
        if(cy < search_top + height/8)
            cmd.linear.x = -0.2;
        else if (cy > search_top + height/8 && cy < search_top+height/4)
            cmd.linear.x = 0;
        else
            cmd.linear.x = 0.2;

//      turn based on error
        cmd.angular.z = -(float)err / 1000;
        publisher.publish(cmd);
    } else {
        //if no moments were created , perform rotatory recovery
        cmd.angular.z = 0.1;
        publisher.publish(cmd);
    }

    //Update GUI
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(1);
}