//
// Created by noetic on 15.06.2020.
//

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    cv::Mat frame;
    cv::namedWindow("window_capture_name", cv::WINDOW_NORMAL);
    frame = cv::imread("/home/noetic/catkin_ws/src/myrobot/myrobot_vision/images/map.pgm");
    if( !frame.data ) { printf("Error loading src image \n"); return -1; }

    cv::Rect roi(cv::Point(frame.size().width/2, 0), cv::Point(frame.size().width - 400, frame.size().height/2));
    cv::Mat mRoi = frame(roi);

    int erosion_size = 1;
    cv::Mat element = getStructuringElement( 2,
                                            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                            cv::Point( erosion_size, erosion_size ) );
    cv::erode( mRoi, mRoi, element );

    cv::Mat exp = mRoi.clone();
    cv::cvtColor(exp, exp, cv::COLOR_BGR2GRAY);
    cv::imwrite("map.pgm", exp);

    cv::imshow("window_capture_name", mRoi);
    cv::waitKey(0);
    return 0;
}