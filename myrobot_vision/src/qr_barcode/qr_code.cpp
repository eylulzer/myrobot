//
// Created by melodic on 6/23/20.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>

typedef struct
{
    std::string type;
    std::string data;
    std::vector <cv::Point> location;
} decodedObject;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void decode(cv::Mat &im, std::vector<decodedObject>&decodedObjects);
void display(cv::Mat &im, std::vector<decodedObject>&decodedObjects);

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_minimum");
    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Subscriber image_sub = imageTransport.subscribe(
            "/camera/rgb/image_raw", 1, imageCallback);

    cv::namedWindow("QRCode");
    ros::spin();
    cv::destroyWindow("QRCode");

    return EXIT_SUCCESS;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvImagePtr;

    try {
        cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Error on CV_Bridge: %s", e.what());
        return;
    }
    cv::Mat im;
    cvImagePtr->image.convertTo(im, -1, 2.0, 50);


    // Variable for decoded objects
    std::vector<decodedObject> decodedObjects;

    // Find and decode barcodes and QR codes
    decode(im, decodedObjects);

    // Display location
    display(im, decodedObjects);

}


// Find and decode barcodes and QR codes
void decode(cv::Mat &im, std::vector<decodedObject>&decodedObjects)
{

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convert image to grayscale
    cv::Mat imGray;
    cvtColor(im, imGray,cv::COLOR_BGR2GRAY);

    // Wrap image data in a zbar image
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);

//    std::cout << "no : " << n << std::endl;

    // Print results
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        decodedObject obj;

        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Print type and data
        std::cout << "Type : " << obj.type << std::endl;
        std::cout << "Data : " << obj.data << std::endl << std::endl;

        // Obtain location
        for(int i = 0; i< symbol->get_location_size(); i++)
        {
            obj.location.emplace_back(symbol->get_location_x(i),symbol->get_location_y(i));
        }

        decodedObjects.push_back(obj);
    }
}

// Display barcode and QR code location
void display(cv::Mat &im, std::vector<decodedObject>&decodedObjects)
{
    // Loop over all decoded objects
    for(auto & decodedObject : decodedObjects)
    {
        std::vector<cv::Point> points = decodedObject.location;
        std::vector<cv::Point> hull;

        // If the points do not form a quad, find convex hull
        if(points.size() > 4)
            convexHull(points, hull);
        else
            hull = points;

        // Number of points in the convex hull
        int n = hull.size();

        for(int j = 0; j < n; j++)
        {
            line(im, hull[j], hull[ (j+1) % n], cv::Scalar(255,255,0), 3);
        }

    }

    // Display results
    cv::imshow("QRCode", im);
    cv::waitKey(3);

}
