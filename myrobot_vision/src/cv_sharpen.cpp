//
// Created by ros on 16.04.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOWS = "Img Window";

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_sharpen");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera_link/image_raw", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOWS);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOWS);

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;
    cv::Mat cvImg;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvImg = cvImagePtr->image;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_bridge: %s", e.what());
        return;
    }

    //do operation over the pic here
    cv::Mat sharpen;
    CV_Assert(cvImagePtr->image.depth() == CV_8U);  // accept only uchar images

    //Using Classic C[] operator
//    sharpen.create(cvImg.size(), cvImg.type());
//    const int nChannels = cvImg.channels();

//    for(int j = 1; j < cvImg.rows - 1; ++j)
//    {
//        const uchar* previous = cvImg.ptr<uchar>(j - 1);
//        const uchar* current  = cvImg.ptr<uchar>(j    );
//        const uchar* next     = cvImg.ptr<uchar>(j + 1);
//
//        uchar* output = sharpen.ptr<uchar>(j);
//
//        for(int i = nChannels; i < nChannels * (cvImg.cols - 1); ++i)
//        {
//            *output++ = cv::saturate_cast<uchar>(5 * current[i]
//                                             -current[i - nChannels] - current[i + nChannels] - previous[i] - next[i]);
//        }
//    }
//
//    sharpen.row(0).setTo(cv::Scalar(0));
//    sharpen.row(sharpen.rows - 1).setTo(cv::Scalar(0));
//    sharpen.col(0).setTo(cv::Scalar(0));
//    sharpen.col(sharpen.cols - 1).setTo(cv::Scalar(0));'


    // using filter 2D -- faster , cleaner
    cv::Mat kern = (cv::Mat_<char>(3, 3) << 0, -1, 0,
                                                        -1, 5, -1,
                                                        0, -1, 0);

    cv::filter2D(cvImg, sharpen, cvImg.depth(), kern);


    //Update gui
    cv::imshow(OPENCV_WINDOWS, sharpen);
    cv::waitKey(3);
}