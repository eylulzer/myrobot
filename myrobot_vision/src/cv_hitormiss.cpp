//
// Created by ros on 29.04.2020.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main() {

    cv::Mat input_image = (
        cv::Mat_<uchar>(8, 8) <<
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 255, 255, 255, 0, 0, 0, 255,
            0, 255, 255, 255, 0, 0, 0, 0,
            0, 255, 255, 255, 0, 255, 0, 0,
            0, 0, 255, 0, 0, 0, 0, 0,
            0, 0, 255, 0, 0, 255, 255, 0,
            0, 255, 0, 255, 0, 0, 255, 0,
            0, 255, 255, 255, 0, 0, 0, 0);

    cv::Mat kernel = (
        cv::Mat_<int>(5, 5) <<
            0, 1, 1, 1, 0,
            0, 1, 1, 1, 0,
            0, 1, 1, 1, 0,
            0, -1, 1, -1, 0,
            0, -1, 1, -1, 0);

    cv::Mat output_image;

    morphologyEx(input_image, output_image, cv::MORPH_HITMISS, kernel);

    const int rate = 50;

    kernel = (kernel + 1) * 127;
    kernel.convertTo(kernel, CV_8U);

    resize(kernel, kernel, cv::Size(), rate, rate, cv::INTER_NEAREST);
    imshow("kernel", kernel);
    cv::moveWindow("kernel", 0, 0);

    resize(input_image, input_image, cv::Size(), rate, rate, cv::INTER_NEAREST);
    imshow("Original", input_image);
    cv::moveWindow("Original", 0, 200);

    resize(output_image, output_image, cv::Size(), rate, rate, cv::INTER_NEAREST);
    imshow("Hit or Miss", output_image);
    cv::moveWindow("Hit or Miss", 500, 200);

    cv::waitKey(0);

    return 0;
}