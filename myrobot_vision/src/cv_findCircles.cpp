//
// Created by ros on 5.05.2020.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

using namespace cv;

/** @function main */
int main(int argc, char** argv)
{
    Mat src, src_gray;

    /// Read the image
    src = cv::imread("/home/ros/catkin_ws/src/myrobot/myrobot_vision/images/circle.png");
    if( !src.data ) { printf("Error loading src image \n"); return -1; }

    /// Convert it to gray
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

    std::vector<Vec3f> circles;

    /// Show your results
    int param1 = 100;
    int param2 = 31;
    int param3 = 5;
    int param4 = 100;
    int min_distance = 15;

    std::string window_name = "Hough Circle";
    namedWindow( window_name , CV_WINDOW_NORMAL );
    createTrackbar("param1", window_name, &param1, 1000);
    createTrackbar("param2", window_name, &param2, 1000);
    createTrackbar("param3", window_name, &param3, 1000);
    createTrackbar("param4", window_name, &param4, 1000);
    createTrackbar("min_distance", window_name, &min_distance, 1000);

    while (true) {
        cv::Mat show;
        src.copyTo(show);
        /// Apply the Hough Transform to find the circles
        HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, min_distance+1, param1+1, param2+1, param3, param4);

        /// Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(show, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(show, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }

        imshow(window_name, show);

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }

    return 0;
}
