#include <ros/ros.h>

#include <pcl_ros/point_cloud.h> // do not delete

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    viewer.showCloud(msg);
    viewer.wasStopped(3); // for updating view
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/points", 1, callback);

    ros::spin();

    return 0;
}