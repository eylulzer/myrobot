//
// Created by ros on 14.05.2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

static const std::string OPENCV_WINDOW = "Match Window";
std::vector<cv::Mat> templArray;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_obj_detect");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);


    for(int i = 1; i<5; i++){
        std::string spath = "/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/templ" +
                            std::to_string(i) + ".png";
        cv::Mat frame = cv::imread(spath, cv::IMREAD_GRAYSCALE);
        if( !frame.data ) { printf("Error loading src image: %s\n", spath.c_str()); continue; }
        templArray.push_back(frame);
    }

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

    cv::Mat graysrc, result_mat;
    cv::cvtColor(cvImagePtr->image, graysrc, cv::COLOR_BGR2GRAY);

    int i = 0;
    for(auto const & templ : templArray){
        i++;
        // method: CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM _CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
        int match_method = CV_TM_CCORR_NORMED;
        cv::matchTemplate(graysrc, templ, result_mat, match_method);
        cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        double minVal;
        double maxVal;
        cv::Point minLoc, maxLoc, matchLoc;
        cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

        if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED) matchLoc = minLoc;
        else matchLoc = maxLoc;

        std::cout << i << " : match loc : " << matchLoc << std::endl;

        cv::rectangle(
            cvImagePtr->image,
            matchLoc,
            cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
            CV_RGB(255, 0, 0),
            3);

//        cv::imshow(std::to_string(i), result_mat);

    }

    //Update GUI
    cv::imshow(OPENCV_WINDOW, cvImagePtr->image);
    cv::waitKey(3);
}