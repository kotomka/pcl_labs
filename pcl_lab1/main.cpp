#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>


using namespace std;

int main()
{
    //upload point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    string path = "/home/pinton/Documents/openCV_labs/PCL_load_lidar_scan/Cloud_1_ds_SOR.ply";
    pcl::io::loadPLYFile(path, *source);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());



////    //1. Passthrought filter

//    pcl::PassThrough<pcl::PointXYZ> pass_x;
//    pass_x.setInputCloud(source);
//    pass_x.setFilterFieldName("x");
//    pass_x.setFilterLimits(22.5,25.0);
//    pass_x.filter(*filtered);

//    pcl::PassThrough<pcl::PointXYZ> pass_y;
//    pass_y.setInputCloud(filtered);
//    pass_y.setFilterFieldName("y");
//    pass_y.setFilterLimits(9.0,29.5);
//    pass_y.filter(*filtered);


//    //visualiztion point cloud


//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> s_handler (source, 255, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> t_handler (filtered, 255, 255, 0);

//    viewer.addPointCloud(source, s_handler, "source");
//    viewer.addPointCloud(filtered, t_handler, "transformed");


//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered");
//    viewer.addCoordinateSystem(1.0, "cloud", 0);


//    while(!viewer.wasStopped()){
//        viewer.spinOnce();
//    }

    return 0;
}
