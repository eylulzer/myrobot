//
// Created by ros on 14.05.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Img Window";

int i = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_save_img");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);
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

    cv::Rect mRoi(100, 100, cvImagePtr->image.size().width - 100, cvImagePtr->image.size().height - 200);
    cv::Mat img = cvImagePtr->image(mRoi);

    //Update GUI
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);

    char key = (char) cv::waitKey(30);

    std::string spath = "/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/templ/templ" +
                        std::to_string(i) + ".png";
    if (key == 'y'){
        imwrite( spath,  img);
        i++;
    }
}