#include <ros/ros.h>

#include <pcl_ros/point_cloud.h> // do not delete

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> copy = pcl::PointCloud<pcl::PointXYZRGB>(*msg);

//    viewer.showCloud(pCopy);
//    viewer.wasStopped(3); // for updating view
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/points", 1, callback);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/ros/catkin_ws/src/myrobot/myrobot_pcl/pcdfiles/cloud_4.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    for (auto & point : cloud->points)
        std::cout << "    " << point.x
                  << " "    << point.y
                  << " "    << point.z
                  << " "    << point.rgb <<
             " "    << int(point.r) <<
             " "    << int(point.g) <<
             " "    << int(point.b) << std::endl;

    viewer.showCloud(cloud);

    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

    ros::spin();

    return 0;
}