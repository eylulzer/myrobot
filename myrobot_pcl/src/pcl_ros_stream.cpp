//
// Created by melodic on 28/05/2020.
//


#include <ros/ros.h>

#include <pcl_ros/point_cloud.h> // do not delete

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


pcl::visualization::PCLVisualizer viewer("PCL Viewer");
bool firstView = true;

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*msg, *cloud);

    if(firstView) {
        viewer.addPointCloud(cloud, "mycloud");
        firstView = false;
    } else
        viewer.updatePointCloud(cloud, "mycloud");

    viewer.spinOnce();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_pcl");
    viewer.setBackgroundColor(0.75, 0.75, 0.75); // light gray
    viewer.spinOnce();

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/points", 1, callback);

    ros::spin();

    return 0;
}