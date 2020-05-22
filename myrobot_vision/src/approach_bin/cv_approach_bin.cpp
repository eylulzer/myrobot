//
// Created by ros on 7.05.2020.
//

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "CustomCircle.h"
#include "cv_approach_bin.h"


void ApproachBin::init() {

    raw_sub = imageTransport.subscribe("/camera/rgb/image_raw", 1, &ApproachBin::imageCallback, this);
    rgb_sub = imageTransport.subscribe("/camera/depth/image_raw", 1, &ApproachBin::depthImageCallback, this);

    this->canGo = std::vector<bool> (3, true);
    obstacleThread = nodeHandle.createTimer(ros::Duration(0.1), &ApproachBin::detectObstacle, this);
    mainThread = nodeHandle.createTimer(ros::Duration(0.1), &ApproachBin::detectCircles, this);

    navigationThread = nodeHandle.createTimer(ros::Duration(0.1), &ApproachBin::circleNavigation, this);
    closeNavThread = nodeHandle.createTimer(ros::Duration(0.1), &ApproachBin::closeNav, this, false, false);

    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    for(int i = 0; i<20; i++){
        std::string spath = "/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/templ/templ" +
            std::to_string(i) + ".png";
        cv::Mat frame = cv::imread(spath);
        if( !frame.data ) { printf("Error loading src image: %s\n", spath.c_str()); continue; }

        //resize
        cv::resize(frame, frame, cv::Size(frame.size() / 4));

        this->templArray.push_back(frame);
    }

    isDone = false;

}

void ApproachBin::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (pointMeasure.camera.center.x == 0.0) {
            // configure camera info
            cv::Size sz = cvImagePtr->image.size();
            pointMeasure.camera.xPixels = sz.width;
            pointMeasure.camera.yPixels = sz.height;
            pointMeasure.camera.center = cv::Point(sz.width / 2, sz.height / 2);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }

    this->rgbImage = cvImagePtr->image;
}

void ApproachBin::depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
    }
    // save on a global variable
    this->depthImage = cvImagePtr->image;
}

void ApproachBin::detectObstacle(const ros::TimerEvent &event) {

    if (this->depthImage.empty()) return;

    int startY = this->pointMeasure.camera.center.y - 5;
    int endY = this->pointMeasure.camera.center.y + 25;
    float region = 0.25; // 25% of image

    // x - 1920 , y - 1080 , %25 = 240px , (960, 540)

    // startX = 840 , 1080
    //          620

    for (int i = -1; i <=1; i++){
        float startX = this->pointMeasure.camera.center.x -
                       (this->pointMeasure.camera.center.x * region)/2  +
                       (i * region * this->pointMeasure.camera.xPixels)/2;

        float endX = this->pointMeasure.camera.center.x +
                     (this->pointMeasure.camera.center.x * region)/2 +
                     (i * region * this->pointMeasure.camera.xPixels)/2;

        double minValue,maxValue;
        cv::Point minPoint, maxPoint;

        cv::Rect rRoi(cv::Point(startX, startY) , cv::Point(endX, endY));
        cv::Mat mRoi = this->depthImage(rRoi);

        mRoi = mRoi.clone();
        mRoi = mRoi.reshape(1, mRoi.rows * mRoi.cols);

        cv::minMaxLoc(mRoi, &minValue, &maxValue, &minPoint, &maxPoint);

//        std::cout << minValue << " : " << maxValue <<
//                  " | " << minPoint << " : " << maxPoint << std::endl;

        // max value : -infinity or up to 5 meters
        // min value : +infinity or range of 0.41 - 5 meters

        this->canGo[i + 1] = !(minValue > 1000.0 || minValue < 1.0);

    }

//    for (const auto b : this->canGo)
//        printf("%s\t", b ? "true" : "false");
//    std::cout << std::endl;
}

void ApproachBin::detectCircles(const ros::TimerEvent &event) {
    mCirclesArray.clear();

    if (rgbImage.empty()) return;
    cv::Mat img = rgbImage.clone();


    for (const auto &item : this->findCircles.methodOne(this->rgbImage)){
//        cv::circle(img, cv::Point(item.x, item.y), item.radius, cv::Scalar(0, 255, 0),  2, cv::LINE_AA);
        this->mCirclesArray.push_back(item);
    }

    for (const auto &item : this->findCircles.methodTwo(this->rgbImage)) {
//        cv::circle(img, cv::Point(item.x, item.y), item.radius, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        this->mCirclesArray.push_back(item);
    }

    for (const auto &item : this->findCircles.methodThree(this->rgbImage)) {
//        cv::circle(img, cv::Point(item.x, item.y), item.radius, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        this->mCirclesArray.push_back(item);
    }

    for (const auto &item : this->findCircles.methodFour(this->rgbImage)){
//        cv::circle(img, cv::Point(item.x, item.y), item.radius, cv::Scalar(255, 255, 0),  2, cv::LINE_AA);
        this->mCirclesArray.push_back(item);
    }


    mCirclesArray = this->meanCircleArray(mCirclesArray);
    this->removeDuplicatesByDepth(mCirclesArray);

//    for(const auto & item: mCirclesArray)
//        cv::circle(img, cv::Point(item.x, item.y), item.radius + 10, cv::Scalar(255,255,255),  0, cv::LINE_AA);

//    imshow("img", img);
    cv::waitKey(3);
}

std::vector<CustomCircle> ApproachBin::meanCircleArray(std::vector<CustomCircle> mcArray) {

    std::vector<CustomCircle> newList;
    for (const auto &v : this->groupByIntersection(mcArray)) {
        if (!v.empty()) {
            CustomCircle total(0, 0, 0);
            for (const auto &cIndex : v) {
                total = CustomCircle(
                    mcArray[cIndex].x + total.x,
                    mcArray[cIndex].y + total.y,
                    mcArray[cIndex].radius + total.radius);
            }

            auto a = float(v.size());
            newList.emplace_back(total.x / a, total.y / a, total.radius / a);
        }
    }

    return newList;
}

std::vector<std::vector<int>> ApproachBin::groupByIntersection(const std::vector<CustomCircle> &mcArray) {

    std::vector<std::vector<int>> intersections(mcArray.size());
    std::vector<int> processedIndex(mcArray.size(), -1);

    for (int i = 0; i < mcArray.size(); i++) {
        if (processedIndex[i] == -1)
            intersections[i].push_back(i);

        for (int j = i + 1; j < mcArray.size(); j++)
            if (processedIndex[i] == -1)
                if (mcArray[i].includesCircle(mcArray[j])) {
                    intersections[i].push_back(j);
                    processedIndex[j] = 0;
                }
    }

    return intersections;
}


void ApproachBin::removeDuplicatesByDepth(std::vector<CustomCircle> &ccArray){
    //get depth info;
    for (CustomCircle &c : ccArray) {
        c.range = this->pointMeasure.getDepthFromPoint(cv::Point2f(c.x, c.y), this->depthImage);
    }

    //sort by x coordinates
    std::sort(ccArray.begin(), ccArray.end(),
              [](const CustomCircle &a, const CustomCircle &b) {
                  return a.x < b.x;
              });

    // Using std::unique to remove duplicates, std::unique needs sorting to work properly
    std::vector<CustomCircle>::iterator ip;
    ip = std::unique(ccArray.begin(), ccArray.end(),
                     [](const CustomCircle &a, const CustomCircle &b) {
                         return a.isInApproxWith(b, 50, 0.1);
                     });

    // Resizing the vector so as to remove the undefined terms
    ccArray.resize(std::distance(ccArray.begin(), ip));

}

void ApproachBin::circleNavigation(const ros::TimerEvent &event) {
    cv::Mat mask = this->maskGreens();
    if(mask.empty()) return;


//  Use the moments() function to calculate the centroid of the blob of the binary image
    cv::Moments M = cv::moments(mask);

    geometry_msgs::Twist cmd;
    cv::Point cntr(0, 0);

    if (M.m00 > 0) {
        cntr.x = int(M.m10 / M.m00);
        cntr.y = int(M.m01 / M.m00);

        // turn based on error and tolerance on angular precision
        float err = (this->pointMeasure.camera.center.x - cntr.x)/5000.0;
        cmd.angular.z = (std::abs(err) < 0.075) ? 0.0 : err;

        float dist = this->pointMeasure.getDepthFromPoint(cntr, depthImage);

        if (dist == -1.0 || std::isnan(dist) || dist > 2.0){
            cmd.linear.x = 0.2;
            publisher.publish(cmd);
        }
        else if (dist <= 2.0) {
            // we come closer than 2 m;
            if(std::abs(cmd.angular.z) < 0.1) cmd.linear.x = 0.0; // wait till it rotates toward the point
            this->getUnderBin(cmd);
        }
    } else {
        //if no moments were created , perform rotatory recovery
        cmd.angular.z = 0.2;
        publisher.publish(cmd);
    }
}

void ApproachBin::getUnderBin(geometry_msgs::Twist & cmd) {

    if (mCirclesArray.size() < 2){
        // no wheels found , stop
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        publisher.publish(cmd);
        return;
    }

    // sort by closest range (distance to camera)
    if (mCirclesArray.size() > 3)
        std::sort(mCirclesArray.begin(), mCirclesArray.end(),
                  [](const CustomCircle &a, const CustomCircle &b) {
                      return a.range < b.range;
                  });

    if (this->pointMeasure.getDistanceFromCustomCircle(mCirclesArray[0], mCirclesArray[1]) > 0.5){
        // robot can go underneath
        this->goal = CustomCircle(
            (mCirclesArray[0].x + mCirclesArray[1].x)/2.0,
            (mCirclesArray[0].y + mCirclesArray[1].y)/2.0,
            0.25, // width of robot
            (mCirclesArray[0].range + mCirclesArray[1].range)/2.0
            );


        // stop other threads
        this->mainThread.stop();
        this->navigationThread.stop();

        //start closeNav
        this->closeNavThread.start();

    } else {
        // cannot enter beneath, stop
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        publisher.publish(cmd);
        return;
    }
}

void ApproachBin::closeNav(const ros::TimerEvent &event) {
    cv::Mat mask = this->maskGreens();
    if(mask.empty()) return;

    geometry_msgs::Twist cmd;

    // Find non zero pixels
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);

    cv::Rect box = cv::boundingRect(pts);
    cv::Point boxCenter((box.tl() + box.br())/2);


    // turn based on error and tolerance on angular precision
    float err = (this->pointMeasure.camera.center.x - boxCenter.x)/2500.0;
    cmd.angular.z = (std::abs(err) < 0.01) ? 0.0 : err;
    std::cout << boxCenter << "\terror: " << err << std::endl <<std::endl;

    cmd.linear.x = (box.br().y > this->pointMeasure.camera.center.y) ? 0.2 : 0.0;

    // image matching
    double conf = this->imageMatched();
    if(conf > 0.7 && conf < 0.8) {
        cmd.linear.x = 0.05;
        cmd.angular.z = 0.0;
    } else if (conf > 0.8) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        this->isDone = true;
        this->closeNavThread.stop();
    }

    publisher.publish(cmd);
    cv::rectangle(this->rgbImage, box, cv::Scalar(0,255,0), 3);
    cv::circle(this->rgbImage, boxCenter, 3, cv::Scalar(0,255,255), -1);
    cv::imshow("close center", this->rgbImage);
    cv::waitKey(3);
}


cv::Mat ApproachBin::maskGreens() {
    if (this->depthImage.empty() || this->rgbImage.empty())
        return cv::Mat();

    cv::Mat hsvImage;
    cvtColor(this->rgbImage, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(18, 53, 9);
    cv::Scalar light_green(91, 159, 73);
    cv::inRange(hsvImage, dark_green, light_green, mask);

    //crop out top of image
    for (int y = 0; y < mask.rows - 2; y++) {
        if (y < 0 || y > mask.rows / 2) {
            for (int x = 0; x < mask.cols; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    return mask;
}

double ApproachBin::imageMatched() {

    cv::Mat result;

    int i = -1;
    for (const auto & templ: templArray){
        i++;
        cv::Mat img;
        cv::resize(this->rgbImage, img, cv::Size(this->rgbImage.size() / 4));

        int result_cols =  img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;

        result.create( result_rows, result_cols, CV_32FC1 );
        int match_method = cv::TM_CCOEFF_NORMED;
        cv::matchTemplate( img, templ, result, match_method);

        double minVal, maxVal;
        cv::Point minLoc, maxLoc, matchLoc;
        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        double dConfidence;
        matchLoc = (match_method == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED) ? minLoc : maxLoc;
        dConfidence = (match_method == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED) ? (1 - minVal) : maxVal;

        if (dConfidence > 0.7){
            std::cout  << i << ". confidence val: " << dConfidence << std::endl;
            rectangle(this->rgbImage, matchLoc*4, cv::Point(matchLoc.x*4 + templ.cols*4, matchLoc.y*4 + templ.rows*4),
                      cv::Scalar(255,255,255), 2, 8, 0);
            return dConfidence;
        }
    }
    return 0.0;
}