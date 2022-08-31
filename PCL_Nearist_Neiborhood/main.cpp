#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

int main()
{
    //upload point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            source(new pcl::PointCloud<pcl::PointXYZ>());
    string path = "/home/robot31-4/Desktop/OpenCV_hello_world2/Cloud_1_ds_SOR.ply";
    pcl::io::loadPLYFile(path, *source);


    pcl::PointXYZ target_point; // задание точки, для которой будем искать
    target_point.x = 1.0;
    target_point.y = 1.0;
    target_point.z = 1.0;

   //Метод грубой силы brute fotce или метод прямого перебора
    float min_dist = FLT_MAX;
    int min_index = 0;
    for(int i = 0 ; i < source ->points.size(); i++) {
        float dist = sqrt(pow(source->points.at(i).x - target_point.x,2)+
                          pow(source->points.at(i).y - target_point.y,2)+
                          pow(source->points.at(i).z - target_point.z,2));
        if(dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    std::cout<<"Nearest point"<<source->points.at(min_index).x<<" "
            <<source->points.at(min_index).y<<""
           <<source->points.at(min_index).z<<"with index" <<min_index<<endl;

    //Построение k-мерного дерева.Поиск ближайшей точки
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree ;
    kdtree.setInputCloud(source);
    int K = 3;
    std::vector<int> min_index_vector(K);
    std::vector<float> min_dist_vector(K);


    int result =
            kdtree.nearestKSearch(target_point, K, min_index_vector, min_dist_vector);
    cout<<"K nearest points"<<endl;
    if(result) {
        for (int i = 0; i < K; i++) {
            cout<<"Point" <<min_index_vector.at(i)<<": "<<source->points.at(min_index_vector.at(i)).x<<
                  " " <<source->points.at(min_index_vector.at(i)).y<<
                  ""<<source->points.at(min_index_vector.at(i)).z <<endl;
        }
    }
 //Радиусный поиск. Поиск в заданном радиусе
    float rad = 2.0;
    min_index_vector.clear();
    min_dist_vector.clear();
     result = kdtree.radiusSearch(target_point, rad, min_index_vector, min_dist_vector);
     cout<<"Radius search"<<endl;
     if(result) {
         for (int i = 0; i < min_index_vector.size(); i++) {
             cout<<"Point" <<min_index_vector.at(i)<<": "<<source->points.at(min_index_vector.at(i)).x<<
                   " " <<source->points.at(min_index_vector.at(i)).y<<
                   ""<<source->points.at(min_index_vector.at(i)).z <<endl;
         }
     }



    //visualiztion point cloud
    pcl::visualization::PCLVisualizer viewer("Test viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> s_handler (source, 255, 0, 0);
    viewer.addPointCloud(source, s_handler, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer.addCoordinateSystem(1.0, "cloud", 0);


    while(!viewer.wasStopped()){
        viewer.spinOnce();

    }

    return 0;
}
