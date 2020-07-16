//
// Created by melodic on 7/15/20.
//

#include "Docking.h"
#include "Line.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <zbar.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>


void Docking::init() {
    this->imageSub = this->imageTransport.subscribe("/camera/rgb/image_raw", 1, &Docking::imageCallback, this);
    this->depthSub = this->imageTransport.subscribe("/camera/depth/image_raw", 1, &Docking::depthCallback, this);
    this->amclSub = this->nodeHandle.subscribe("/amcl_pose", 1, &Docking::amclPose, this);
    this->cmdPub = nodeHandle.advertise<geometry_msgs::Twist>("/diffagv_diff_drive_controller/cmd_vel", 10);
    this->decodeThread = this->nodeHandle.createTimer(ros::Duration(0.1), &Docking::decode, this);
    this->navThread = this->nodeHandle.createTimer(ros::Duration(0.1), &Docking::moveQR, this);
}

void Docking::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    this->rgbImage = cvImagePtr->image;
}

void Docking::depthCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }

    this->depthImage = cvImagePtr->image;
}

void Docking::amclPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    robotPosition = msg;
    std::cout << "\n" << msg->pose.pose.position.x
              << " : " << msg->pose.pose.position.y
              << " : " << msg->pose.pose.orientation.w
              << " : " << msg->pose.pose.orientation.z
              << std::endl;
}

void Docking::decode(const ros::TimerEvent &event) {

    if (this->rgbImage.empty())
        return;

    // add brightness and contrast
    // Convert image to grayscale
    cv::Mat imGray;
    this->rgbImage.convertTo(imGray, -1, 2.0, 50);
    cvtColor(imGray, imGray, cv::COLOR_BGR2GRAY);

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Wrap image data in a zbar image
    zbar::Image image(this->rgbImage.cols, this->rgbImage.rows, "Y800", (uchar *) imGray.data,
                      this->rgbImage.cols * this->rgbImage.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);
    // std::cout << "no : " << n << std::endl;

    this->decodedObjects.clear();
    // stack the results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        decodedObject obj;

        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Obtain location
        for (int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        this->decodedObjects.push_back(obj);
    }
}

void Docking::moveQR(const ros::TimerEvent &event) {
    if (this->rgbImage.empty())
        return;

    cv::Mat im = this->rgbImage.clone();

    // Loop over all decoded objects
    for (auto &decodedObject : this->decodedObjects) {

        if (decodedObject.type != this->powerGoal.type ||
            decodedObject.data != this->powerGoal.qrData)
            continue;

        std::vector<cv::Point> quad = decodedObject.location;
        std::vector<Line> quadLines, verticalLines;

        int n = quad.size();

        for (int j = 0; j < n; j++) {
            quadLines.emplace_back(
                    cv::Point(quad[j].x, quad[j].y),
                    cv::Point(quad[(j + 1) % n].x, quad[(j + 1) % n].y)
            );

            if (std::fabs(quadLines[j].slope) > 0.5)
                verticalLines.emplace_back(quadLines[j]);
        }

        //draw
        if (verticalLines.size() == 2) {

            cv::rectangle(
                    im,
                    verticalLines[0].start,
                    verticalLines[1].start,
                    cv::Scalar(255, 0, 255),
                    4
            );


            cv::Point center = cv::Point(
                    (verticalLines[0].start.x + verticalLines[1].start.x) / 2,
                    (verticalLines[0].start.y + verticalLines[1].start.y) / 2
            );

            cv::circle(im, center, 5, cv::Scalar(0, 0, 255), -1);

            std::cout << std::endl;
            for (const auto &verticalLine: verticalLines) {
                std::cout << verticalLine.toString() << std::endl;
                cv::line(im, verticalLine.start, verticalLine.end, cv::Scalar(0, 255, 0), 2);
            }

            float dist = this->pointMeasure.getDepthFromPoint(center, this->depthImage);
            std::cout << "Distance to QR CODE: " << dist << std::endl;

            geometry_msgs::Twist cmd;
            if (dist > this->powerGoal.distanceToQR + 0.1) {

                // turn based on error and tolerance on angular precision
                float err = (this->pointMeasure.camera.center.x - center.x) / 1000.0;

                if (std::fabs(err) > 0.02) {
                    cmd.angular.z = err;
                    std::cout << "ERROR: " << err << std::endl;

                }
            }

            if (dist > this->powerGoal.distanceToQR + 0.2) { //35cm

                cmd.linear.x = -0.2;
            } else if (dist > this->powerGoal.distanceToQR){

                cmd.linear.x = -0.1;
            } else if (dist < this->powerGoal.distanceToQR - 0.05) { //10cm


                cmd.linear.x = 0.1;
            }

            this->cmdPub.publish(cmd);

            if (dist < this->powerGoal.distanceToQR + 0.1 && dist < this->powerGoal.distanceToQR)
                if (std::abs(verticalLines[0].length - verticalLines[1].length) > this->powerGoal.qrVerticalTolerance)
                    std::cout << "failed " << std::endl;

        }

    }

    // Display results
    cv::imshow(this->windowName, im);
    cv::waitKey(3);

}
