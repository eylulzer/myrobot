//
// Created by melodic on 28/05/2020.
//

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/melodic/catkin_ws/src/myrobot/myrobot_pcl/pcdfiles/fragment.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

// Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix;
// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

// Estimate the XYZ centroid
    pcl::compute3DCentroid(*cloud, xyz_centroid);

// Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*cloud, xyz_centroid, covariance_matrix);

    pcl::flipNormalTowardsViewpoint (cloud->points.at(0), 0.0, 0.0, 0.0, xyz_centroid);

    std::cout << covariance_matrix << std::endl;

    pcl::visualization::PCLVisualizer viewer ("Covariances");

    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(cloud, "original_cloud");

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


}