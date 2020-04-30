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
image_transport::Subscriber image_sub;
static const std::string OPENCV_WINDOW = "Img Window";

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_pyramids");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_sub = imageTransport.subscribe(
        "/camera_link/image_raw", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
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
    image_sub.shutdown();
    //do operation over image
    cv::Mat src = cvImagePtr->image;

    while(true){
        imshow( OPENCV_WINDOW, src );
        char c = (char)cv::waitKey(0);
        if( c == 27 )
            break;
        else if( c == 'i' )
            pyrUp( src, src, cv::Size( src.cols*2, src.rows*2 ) );
        else if( c == 'o' )
            pyrDown( src, src, cv::Size( src.cols/2, src.rows/2 ) );
    }

}