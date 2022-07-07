#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

void Clustering_process(const sensor_msgs::PointCloud2ConstPtr& aft_ransac){
    RT1.start();
    PCXYZI tmp;
    pcl::fromROSMsg(*aft_ransac,tmp);
    PCXYZI::Ptr upsampledCloud (new PCXYZI);;
    copyPointCloud(tmp,*upsampledCloud);
    //-----------NoiseFiltering-----------
    PCXYZI::Ptr filteredCloud (new PCXYZI); //saving spcae for filteredData
    if( switch_NoiseFiltering ) NoiseFiltering( upsampledCloud, filteredCloud); //call NoiseFiltering func  ////  current : on
    else filteredCloud = upsampledCloud;
    //-----------clustering------------
    PCXYZI Fin_Cloud;
    Clustering(filteredCloud, Fin_Cloud, switch_DBscan, switch_Euclid);
    RT1.end_cal("clustering");
    FPS1.update();
    //cout << "-------------------------------------------------" << endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Clustering");    //node name 
	ros::NodeHandle nh;                     //nodehandle    

    nh.getParam("/Clustering_node/switch_NoiseFiltering", switch_NoiseFiltering);
    //Euclid
    nh.getParam("/Clustering_node/switch_Euclid", switch_Euclid);
    nh.getParam("/Clustering_node/EC_eps", EC_eps);
    nh.getParam("/Clustering_node/EC_MinClusterSize", EC_MinClusterSize);
    nh.getParam("/Clustering_node/EC_MaxClusterSize", EC_MaxClusterSize);
    //DBSCAN
    nh.getParam("/Clustering_node/switch_DBscan", switch_DBscan);
    nh.getParam("/Clustering_node/DBscan_eps", DBscan_eps);
    nh.getParam("/Clustering_node/DBscan_minPts", DBscan_minPts);
    nh.getParam("/Clustering_node/DB_MinClusterSize", DB_MinClusterSize);
    nh.getParam("/Clustering_node/DB_MaxClusterSize", DB_MaxClusterSize);
    //etc
    nh.getParam("/Clustering_node/switch_jiwon_filter", switch_jiwon_filter);
    nh.getParam("/Clustering_node/switch_DY_filter", switch_DY_filter);
    nh.getParam("/Clustering_node/Ransac_Z_ROI", Ransac_Z_ROI);  //DY_filter param
    nh.getParam("/Clustering_node/REMOVE_FACTOR", REMOVE_FACTOR);

	//ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points_resampling", 1, Clustering_process);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_1_velodyne_points_ransac", 1, Clustering_process);
    pub_Clu = nh.advertise<sensor_msgs::PointCloud2> ("/3_velodyne_points_Clustering", 1);
    pub_msg = nh.advertise<std_msgs::String> ("/Lidar_msg", 1); 

    pub_obj = nh.advertise<Lidar_3DOD_2022::obj_msg> ("/Lidar_obj", 1);
    pub_object = nh.advertise<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_object", 1);
    //<패키지 명/메시지 파일 명>

    ros::spin();
}
