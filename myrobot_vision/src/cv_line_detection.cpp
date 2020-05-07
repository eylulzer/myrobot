//
// Created by ros on 30.04.2020.
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
    imshow("gray", gray);
    waitKey(0);

    // Apply adaptiveThreshold at the bitwise_not of gray, notice the ~ symbol
    Mat bw;
    adaptiveThreshold(~gray, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, -2);

    // Show binary image
    imshow("binary", bw);
    waitKey(0);

    // Create the images that will use to extract the horizontal and vertical lines
    Mat horizontal = bw.clone();
    Mat vertical = bw.clone();

    // Specify size on horizontal axis
    int horizontal_size = horizontal.cols/15; // change to 30 and see how an extra line appears

    // Create structure element for extracting horizontal lines through morphology operations
    Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontal_size, 1));

    // Apply morphology operations
    erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));

    // Show extracted horizontal lines
    imshow("horizontal", horizontal);
    waitKey(0);

    // Specify size on vertical axis
    int vertical_size = vertical.rows / 15;

    // Create structure element for extracting vertical lines through morphology operations
    Mat verticalStructure = getStructuringElement(MORPH_ELLIPSE, Size(1, vertical_size));

    // Apply morphology operations
    erode(vertical, vertical, verticalStructure, Point(-1, -1));
    dilate(vertical, vertical, verticalStructure, Point(-1, -1));

    // Show extracted vertical lines
    imshow("vertical", vertical);
    waitKey(0);

    // Inverse vertical image
    bitwise_not(vertical, vertical);
    imshow("vertical_bit", vertical);
    waitKey(0);

    // 1. extract edges
    Mat edges;
    adaptiveThreshold(vertical, edges, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, -2);
    imshow("edges", edges);
    waitKey(0);

    // 2. dilate(edges)
    Mat kernel = Mat::ones(2, 2, CV_8UC1);
    dilate(edges, edges, kernel);
    imshow("dilate", edges);
    waitKey(0);

    // 3. src.copyTo(smooth)
    Mat smooth;
    vertical.copyTo(smooth);

    // 4. blur smooth img
    blur(smooth, smooth, Size(2, 2));

    // 5. smooth.copyTo(src, edges)
    smooth.copyTo(vertical, edges);

    // Show final result
    imshow("smooth - final", vertical);
    waitKey(0);

    return 0;
}
