//
// Created by ros on 6.05.2020.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    //! [load_image]
    Mat src = cv::imread("/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/notes.png");
    if( !src.data ) { printf("Error loading src image \n"); return -1; }

    imshow("original", src);

    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    waitKey(0);

    Mat bw, cnts;
    vector<vector<cv::Point> > contours;
    adaptiveThreshold(gray, bw, 255, THRESH_OTSU, THRESH_BINARY_INV, 15, 0);

    Mat horizontal = bw.clone();

    // Create structure element for extracting horizontal lines through morphology operations
    Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontal.cols/30, 1));

    erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));

    morphologyEx(horizontal, horizontal, MORPH_OPEN, horizontalStructure, cv::Point(-1,-1), 2);
    findContours(horizontal, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    drawContours(src, contours, -1, Scalar(0,255,0), 6);
    imshow("gray", src);
//    for(const auto &c: contours)
/*
 *

# Remove horizontal
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25,1))
    detected_lines = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, horizontal_kernel, iterations=2)
    cnts = cv2.findContours(detected_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
    cv2.drawContours(image, [c], -1, (255,255,255), 2)

# Repair image
    repair_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,6))
    result = 255 - cv2.morphologyEx(255 - image, cv2.MORPH_CLOSE, repair_kernel, iterations=1)

    cv2.imshow('thresh', thresh)
    cv2.imshow('detected_lines', detected_lines)
    cv2.imshow('image', image)
    cv2.imshow('result', result)
    cv2.waitKey()
    */

    return 0;
}