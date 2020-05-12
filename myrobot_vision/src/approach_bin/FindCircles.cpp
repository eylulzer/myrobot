//
// Created by ros on 7.05.2020.
//

#include "FindCircles.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "CustomCircle.h"

std::vector<CustomCircle> FindCircles::methodOne(const cv::Mat &image) {
    std::vector<CustomCircle> circlesArray;
    if (image.empty()) return circlesArray;

    //do operation over image
    cv::Mat hsvImage, dst, gray;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // closing
    int morph_size = 15;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(hsvImage, hsvImage, 3, element);

    // Threshold the HSV image, keep only the green pixels
    cv::Mat mask;
    cv::Scalar dark_green(10, 50, 10);
    cv::Scalar light_green(80, 255, 80);
    cv::inRange(hsvImage, dark_green, light_green, mask);

    // put black over hsvImage bin mask
    cv::Mat black = cv::Mat::zeros(hsvImage.size(), hsvImage.type());
    black.copyTo(hsvImage, mask);

    // bottom part of image
    cv::Point cropOrigin(0, image.rows / 2 - 20);
    cv::Size s(image.cols, image.rows - (image.rows / 2 - 20));
    cv::Mat im = hsvImage(cv::Rect(cropOrigin, s));
    im = im.clone();

    //blur
    cv::medianBlur(im, dst, 5);

    //sharpen
    cv::Mat kern = (cv::Mat_<char>(3, 3) <<
                                         0, -1, 0,
        -1, 5, -1,
        0, -1, 0);

    cv::filter2D(dst, dst, dst.depth(), kern);

    //add brightness
    cv::Mat toCanny, edges;
    dst.convertTo(toCanny, -1, 2, 10);

    //canny edge detector
    int lowThreshold = 90;
    Canny(toCanny, edges, lowThreshold, lowThreshold * 3, 3);
    dst = cv::Scalar::all(0);
    im.copyTo(dst, edges);


    // black n white
    cvtColor(dst, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> cA;
    cv::HoughCircles(
        gray, cA, cv::HOUGH_GRADIENT, 1,
        gray.rows / 12, 100, 31, 10, 100
    );

    for (int i = 0; i < cA.size(); i++)
        circlesArray.push_back(CustomCircle(cA[i][0], cA[i][1] + (image.rows / 2 - 20), cA[i][2]));

    return circlesArray;
}

std::vector<CustomCircle> FindCircles::methodTwo(const cv::Mat &image) {
    std::vector<CustomCircle> circlesArray;
    if (image.empty()) return circlesArray;

    //do operation over image
    cv::Mat hsvImage, dst, gray, mask;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // closing
    int morph_size = 10;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(hsvImage, hsvImage, 3, element);

    // Threshold the HSV image
    cvtColor(hsvImage, hsvImage, CV_BGR2HSV);
    cv::inRange(hsvImage, cv::Scalar(85, 128, 96), cv::Scalar(138, 200, 125), mask);

    // put black over hsvImage bin mask
    cv::Mat black = cv::Mat::zeros(hsvImage.size(), hsvImage.type());
    black.copyTo(hsvImage, ~mask);

    //blur
    cv::medianBlur(hsvImage, dst, 15);

    //sharpen
    cv::Mat kern = (cv::Mat_<char>(3, 3) << 0, -1, 0,
        -1, 5, -1,
        0, -1, 0);

    cv::filter2D(dst, dst, dst.depth(), kern);

    //add brightness
    dst.convertTo(gray, -1, 3, 20);

    // black n white
    cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);


    std::vector<cv::Vec3f> cA;
    cv::HoughCircles(
        gray, cA, cv::HOUGH_GRADIENT, 1,
        gray.rows / 12, 100, 31, 10, 100
    );

    for (auto & i : cA)
        circlesArray.emplace_back(i[0], i[1], i[2]);
}

std::vector<CustomCircle> FindCircles::methodThree(const cv::Mat &image) {
    std::vector<CustomCircle> circlesArray;
    if (image.empty()) return circlesArray;

    cv::Mat hsvImage, dst, gray, mask;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // closing
    int morph_size = 15;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(hsvImage, hsvImage, 3, element);

    // Threshold the HSV image
    cvtColor(hsvImage, hsvImage, CV_BGR2HSV);
    cv::inRange(hsvImage, cv::Scalar(85, 128, 96), cv::Scalar(138, 200, 125), mask);

    // put black over hsvImage bin mask
    cv::Mat black = cv::Mat::zeros(hsvImage.size(), hsvImage.type());
    black.copyTo(hsvImage, ~mask);

    //sharpen
    cv::Mat kern = (cv::Mat_<char>(3, 3) << 0, -1, 0,
        -1, 5, -1,
        0, -1, 0);

    cv::filter2D(hsvImage, dst, dst.depth(), kern);

    //add brightness
    dst.convertTo(gray, -1, 3, 20);

    // black n white
    cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blur, canny_output;
    cv::blur(gray, blur, cv::Size(5, 5));
    cv::Canny(blur, canny_output, 100, 100 * 2);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        minEnclosingCircle(contours_poly[i], center[i], radius[i]);
    }

    for (size_t i = 0; i < contours.size(); i++)
        if (center[i].y >= image.rows / 2)
            circlesArray.push_back(CustomCircle(center[i].x, center[i].y, radius[i]));

    return circlesArray;
}

std::vector<CustomCircle> FindCircles::methodFour(const cv::Mat &image) {
    std::vector<CustomCircle> circlesArray;
    if (image.empty()) return circlesArray;

    cv::Mat im;

    // closing
    int morph_size = 15;
    cv::Mat element = getStructuringElement(2, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                            cv::Point(morph_size, morph_size));
    cv::morphologyEx(image, im, 3, element);

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 100;
    params.filterByCircularity = true;
    params.minCircularity = 0.05;
    params.filterByConvexity = true;
    params.minConvexity = 0.1;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(im, keypoints);

    for (const auto &keypoint: keypoints)
        if (keypoint.pt.y >= image.rows / 2.0)
            circlesArray.emplace_back(CustomCircle(keypoint.pt.x, keypoint.pt.y, keypoint.size / 2.0));

    return circlesArray;
}
