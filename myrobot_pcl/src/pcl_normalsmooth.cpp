//
// Created by melodic on 28/05/2020.
//

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/melodic/catkin_ws/src/myrobot/myrobot_pcl/pcdfiles/fragment.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    std::cout << cloud->width << " : " << cloud->height << " : " << cloud->size() << std::endl;

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    // need organized dataset in order to work
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (cloud, 230, 20, 20);
    viewer.addPointCloud(cloud, "mycloud");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}