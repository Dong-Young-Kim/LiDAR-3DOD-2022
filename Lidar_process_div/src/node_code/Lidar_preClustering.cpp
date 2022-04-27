/*
#include <Lidar_process_div/Lidar_declare.h>

using namespace std;

void preClustering_process(const sensor_msgs::PointCloud2ConstPtr& aft_ROI){
    PCXYZI rawData;
    pcl::fromROSMsg(*aft_ROI,rawData);
    PCXYZI::Ptr downsampledCloud (new PCXYZI); //saving spcae for downsampledData
    if( switch_DownSampling ) DownSampling(rawData,downsampledCloud); //call DownSampling func
    else copyPointCloud(rawData,*downsampledCloud);
    //-----------pre_NoiseFiltering-----------
    PCXYZI::Ptr prefilteredCloud (new PCXYZI); //saving spcae for filteredData
    if( switch_preNoiseFiltering ) NoiseFiltering(downsampledCloud,prefilteredCloud, "pre"); //call NoiseFiltering func  ////  current : off
    else prefilteredCloud = downsampledCloud;
    //-----------pre_Clustering-----------
    PCXYZI TotalCloud; //possible to put in func, but... for the enpandability
    if( switch_pre_Clustering ) EuclideanClustering(prefilteredCloud,TotalCloud);
    else {//얘는 꺼져도 나오게 해야함. 그래야 pub해서 다음 노드한테 줄 수 있음
        TotalCloud = *prefilteredCloud;
        sensor_msgs::PointCloud2 output; 
        pub_process(TotalCloud,output);
        pub_Clu1.publish(output); 
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "preClustering"); //node name 
	ros::NodeHandle nh;         //nodehandle
    nh.getParam("/preClustering_node/switch_preNoiseFiltering", switch_preNoiseFiltering);
    nh.getParam("/preClustering_node/switch_DownSampling", switch_DownSampling);
    nh.getParam("/preClustering_node/switch_pre_Clustering", switch_pre_Clustering);
    nh.getParam("/preClustering_node/voxel_size_x", voxel_size_x);
    nh.getParam("/preClustering_node/voxel_size_y", voxel_size_y);
    nh.getParam("/preClustering_node/voxel_size_z", voxel_size_z);
    nh.getParam("/preClustering_node/EC_eps", EC_eps);
    nh.getParam("/preClustering_node/EC_MinClusterSize", EC_MinClusterSize);
    nh.getParam("/preClustering_node/EC_MaxClusterSize", EC_MaxClusterSize);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/1_velodyne_points_ROI", 100, preClustering_process);
    pub_Clu1 = nh.advertise<sensor_msgs::PointCloud2> ("/2_velodyne_points_preClustered", 1);

    
	ros::spin();
}
*/