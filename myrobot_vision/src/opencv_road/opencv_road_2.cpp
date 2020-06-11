//
// Created by melodic on 04/06/2020.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <geometry_msgs/Twist.h>
#include "MyLine.h"

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

cv::Mat getROI(const cv::Mat &mRoi, cv::Point roi_points[], int n);

double imageMatched(cv::Mat &rgbImage);

cv::Mat templ;

bool target_reached = false;

ros::Publisher pub;

int main(int argc, char **argv) {

    ros::init(argc, argv, "opencv_eroding");
    ros::NodeHandle nodeHandle;
    pub = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
        "/camera/rgb/image_raw", 1, imageCallback);


    templ = cv::imread("/home/melodic/catkin_ws/src/myrobot/myrobot_vision/images/door.png");
    if (!templ.data) {
        printf("Error loading src image \n");
        return -1;
    }

    cv::namedWindow("OPENCV_WINDOW");
    ros::spin();
    cv::destroyAllWindows();

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    geometry_msgs::Twist cmd;
    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    cv::Mat original = cvImagePtr->image.clone();

    cv::Point origin = cv::Point(0, original.size().height / 2);
    cv::Point size = cv::Point(original.size().width, original.size().height);
    cv::Rect r = cv::Rect(origin, size);
    cv::Mat mRoi = original(r);

    //add contrast
    mRoi.convertTo(mRoi, -1, 2, 10);

    int erosion_size = 9;
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                            cv::Point(erosion_size, erosion_size));


    //blur, erosion, blur, sharpen; make patterns dissapear, leave only edges
    cv::blur(mRoi, mRoi, cv::Size(20, 20));
    cv::erode(mRoi, mRoi, element);
    cv::medianBlur(mRoi, mRoi, 35);

    cv::Mat mask;

    cvtColor(mRoi, mRoi, cv::COLOR_BGR2HSV);
    inRange(mRoi, cv::Scalar::all(0), cv::Scalar(1, 1, 70), mask);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_poly(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
    }

    cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(drawing, contours_poly, -1, cv::Scalar::all(255), -1);

    cv::Point roi_pts[4];

    roi_pts[0] = cv::Point(drawing.size().width * 0.05, drawing.size().height * 0.30); // tl
    roi_pts[1] = cv::Point(drawing.size().width * 0.01, drawing.size().height * 0.95); // bl
    roi_pts[2] = cv::Point(drawing.size().width * 0.99, drawing.size().height * 0.95); // br
    roi_pts[3] = cv::Point(drawing.size().width * 0.95, drawing.size().height * 0.30); // tr

    mRoi = getROI(drawing, roi_pts, 4);
    erode(mRoi, mRoi, element);

    cv::Moments M = cv::moments(mRoi);
    cv::Point mRoi_center(0, 0);

    cmd.linear.x = 0.2;
    if (M.m00 > 0) {
        mRoi_center.x = int(M.m10 / M.m00);
        mRoi_center.y = int(M.m01 / M.m00);

        if (mRoi_center.y > mRoi.size().height - mRoi.size().height * 0.1)
            cmd.angular.z = 0.0;
        else {
            mRoi_center.y = int(M.m01 / M.m00) + mRoi.size().height;

            cv::circle(cvImagePtr->image, mRoi_center, 20, cv::Scalar(0, 255, 100), -1);

            float err = (mRoi.size().width / 2 - mRoi_center.x) / 1000.0;
            cmd.angular.z = (std::abs(err) < 0.075) ? 0.0 : err;
        }
    } else if (imageMatched(cvImagePtr->image) > 0.6) {
        cmd.linear.x = 0.0;
        pub.publish(cmd);
    }

    if (!target_reached)
        pub.publish(cmd);

    cv::drawContours(cvImagePtr->image,
                     contours_poly,
                     -1,
                     cv::Scalar(0, 0, 255),
                     3,
                     cv::LINE_8,
                     cv::_InputArray(),
                     0x7fffffff,
                     cv::Point(0, original.size().height / 2)
    );

    cv::resize(cvImagePtr->image, cvImagePtr->image, cv::Size(), 0.5, 0.5);
    cv::imshow("OPENCV_WINDOW", cvImagePtr->image);
    cv::waitKey(3);
}


cv::Mat getROI(const cv::Mat &mRoi, cv::Point roi_points[], int n) {

    cv::Mat mask = cv::Mat::zeros(mRoi.size(), mRoi.type());
    cv::Mat black = cv::Mat::zeros(mRoi.size(), mRoi.type());

    const cv::Point *ppt[1] = {roi_points};
    int npt[] = {n};
    cv::fillPoly(black, ppt, npt, 1, cv::Scalar::all(255));

    mRoi.copyTo(mask, black);
    return mask;
}

double imageMatched(cv::Mat &rgbImage) {
    if (rgbImage.empty())
        return 0.0;

    cv::Mat result;

    int result_cols = rgbImage.cols - templ.cols + 1;
    int result_rows = rgbImage.rows - templ.rows + 1;

    result.create(result_rows, result_cols, CV_32FC1);
    int match_method = cv::TM_CCOEFF_NORMED;
    cv::matchTemplate(rgbImage, templ, result, match_method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    double dConfidence;
    matchLoc = maxLoc;
    dConfidence = maxVal;

    if (dConfidence > 0.5) {
        std::cout << ". confidence val: " << dConfidence << std::endl;
        rectangle(rgbImage, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
                  cv::Scalar::all(255), 4, 8, 0);
        target_reached = true;
        return dConfidence;
    }

    return 0.0;
}