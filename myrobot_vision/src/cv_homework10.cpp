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

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Img Window";
ros::Publisher publisher;

cv::Point center = cv::Point(100,100);
float depthInfo = 1.0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_bin");
    ros::NodeHandle nodeHandle;
    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);

    image_transport::Subscriber rawimage_sub = imageTransport.subscribe(
        "/camera/depth/image_raw", 1, depthImageCallback);

    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
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
        return;
    }

    depthInfo = cvImagePtr->image.at<float>(center);
    std::cout << depthInfo <<std::endl;

//    std::cout << cvImagePtr->image.at<float>(center) << "\t" << center << std::endl;

//
//    float min_range_ = 0.5;
//    float max_range_ = 5.5;
//
//    // convert to something visible
//    cv::Mat img(depthImg.rows, depthImg.cols, CV_8UC1);
//
//    for(int i = 0; i < depthImg.rows; i++)
//    {
//        float* Di = depthImg.ptr<float>(i); // row float
//        char* Ii = img.ptr<char>(i);
//        for(int j = 0; j < depthImg.cols; j++)
//        {
//            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
//        }
//    }
//
//    // display
//    cv::imshow("WINDOW_NAME", img);
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
    cv::Mat image = cvImagePtr->image;;
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    imshow("hsv", hsvImage);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(10, 70, 10);
    cv::Scalar light_green(80, 255, 80);
    cv::inRange(hsvImage, dark_green, light_green, mask);
    imshow("mask", mask);

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
        center = cv::Point(cx, cy);
        cv::circle(image, center, 20, CV_RGB(255, 0, 0), -1);

//      Move the robot in proportion to the error signal
        int err = cx - width / 2;

//      go forward towards the center of the dumpster
//      if too close, dumpster green will be left in the first eighth of image, go back
//      if in between , stop
        if(cy < search_top + height/8)
            cmd.linear.x = -0.2;
        else if (cy > search_top + height/8 && cy < search_top+height/4)
            cmd.linear.x = (depthInfo < 1.1) ? 0.0 : 0.1; // use depth info to get more accuracy
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