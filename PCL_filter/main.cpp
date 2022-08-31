#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
    //upload point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            source(new pcl::PointCloud<pcl::PointXYZ>());
    string path = "/home/pinton/Documents/openCV_labs/PCL_load_lidar_scan/Cloud_1_ds_SOR.ply";
    pcl::io::loadPLYFile(path, *source);

    //transformation point cloud

    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
    float theta = 1;
    transform_mat(0,0) = cos(theta);
    transform_mat(0,1) = -sin(theta);
    transform_mat(1,0) = sin(theta);
    transform_mat(1,1) = cos(theta);


    transform_mat(0,3) = 10;

    cout<<"Current transformation"<<endl<<transform_mat<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source, *transformed, transform_mat);




    //visualiztion point cloud
    pcl::visualization::PCLVisualizer viewer("Test viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> s_handler (source, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> t_handler (source, 255, 255, 0);

    viewer.addPointCloud(source, s_handler, "source");
    viewer.addPointCloud(transformed, t_handler, "transformed");


    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");
    viewer.addCoordinateSystem(1.0, "cloud", 0);


    while(!viewer.wasStopped()){
        viewer.spinOnce();

    }

    return 0;
}
