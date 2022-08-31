#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main()
{
    //upload point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            source(new pcl::PointCloud<pcl::PointXYZ>());
    string path = "/home/robot31-4/Desktop/OpenCV_hello_world2/Cloud_1_ds_SOR.ply";
    pcl::io::loadPLYFile(path, *source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            filtered(new pcl::PointCloud<pcl::PointXYZ>());

    //1. Passthrough фильтр
   // pcl::PassThrough<pcl::PointXYZ> pass;
    //pass.setInputCloud(source);
   // pass.setFilterFieldName("z");  //ось
    //pass.setFilterLimits(0.0, 1.0); //границы
   // pass.filter(*filtered);

    //Воксельный фильтр
    //pcl::VoxelGrid<pcl::PointXYZ> voxGrid;
    //voxGrid.setInputCloud(source);
    //voxGrid.setLeafSize(0.1, 0.1, 0.1);
   // voxGrid.filter(*filtered);

    //Радиусный фильтр
    //pcl::RadiusOutlierRemoval<pcl::PointXYZ> radfilt;
    //radfilt.setInputCloud(source);
    //radfilt.setRadiusSearch(0.1); //радиус 10 см
    //radfilt.setMinNeighborsInRadius(20); //кол-во точек в радиусе
    //radfilt.filter(*filtered);

    //Статистический фильтр
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat ;
    stat.setInputCloud(source); //из какого облака точек
    stat.setMeanK(50); //кол-во ближайших соседей
    stat.setStddevMulThresh(1.0);//коэффициент при отклонении
    stat.filter(*filtered);



    //visualiztion point cloud
    pcl::visualization::PCLVisualizer viewer("Test viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> s_handler (source, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> t_handler (filtered, 255, 255, 0);

    //viewer.addPointCloud(source, s_handler, "source");
    viewer.addPointCloud(filtered, t_handler, "filtered");


    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered");
    viewer.addCoordinateSystem(1.0, "cloud", 0);


    while(!viewer.wasStopped()){
        viewer.spinOnce();

    }

    return 0;
}
