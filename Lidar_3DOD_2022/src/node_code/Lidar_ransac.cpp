#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

void ransac_process(const sensor_msgs::PointCloud2ConstPtr& aft_ROI){
    RT1.start();
    PCXYZI tmp;
    pcl::fromROSMsg(*aft_ROI,tmp);
    PCXYZI TotalCloud;
    copyPointCloud(tmp,TotalCloud);

    //-----------UpSampling-----------
    PCXYZI::Ptr upsampledCloud (new PCXYZI);
    if( switch_UpSampling ) UpSampling(TotalCloud,upsampledCloud);
    else *upsampledCloud = TotalCloud;

    //----------ransac----------
    if( switch_RanSaC ) RanSaC(upsampledCloud);
    else{
        sensor_msgs::PointCloud2 output; 
        pub_process(*upsampledCloud, output); 
        pub_RS.publish(output); 
    }
    
    RT1.end_cal("ransac");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ransac"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/ransac_node/switch_UpSampling", switch_UpSampling);
    nh.getParam("/ransac_node/switch_RanSaC", switch_RanSaC);
    nh.getParam("/ransac_node/ransac_distanceThreshold", ransac_distanceThreshold);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/1_velodyne_points_ROI", 1, ransac_process);
    pub_RS = nh.advertise<sensor_msgs::PointCloud2> ("/2_1_velodyne_points_ransac", 1);
    pub_GND = nh.advertise<sensor_msgs::PointCloud2> ("/2_2_velodyne_points_ground", 1);
    
	ros::spin();
}
