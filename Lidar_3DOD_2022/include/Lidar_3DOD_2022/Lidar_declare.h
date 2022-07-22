#pragma once

#ifndef LIDAR_DECLARE
#define LIDAR_DECLARE
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Lidar_3DOD_2022/SDW_DBSCAN.h>
//#include <Lidar_process_div/DBSCAN_hanbin.h>
#include "Lidar_3DOD_2022/obj_msg.h"  //include "패키지 명/메시지 파일 명.h"
#include "Lidar_3DOD_2022/object_msg_arr.h"  //include "패키지 명/메시지 파일 명.h"


//---------------------------------------------------
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointXYZI PXYZI;

ros::Publisher pub_ROI;     //ROI
ros::Publisher pub_RS;      //ransac
ros::Publisher pub_GND;     //ground
ros::Publisher pub_Clu;
ros::Publisher pub_msg;
ros::Publisher pub_obj;     //user defined msg
ros::Publisher pub_object;     //user defined msg
//ros::Publisher OUT_MSG;     //out message

double REMOVE_FACTOR;
float voxel_size_x, voxel_size_y, voxel_size_z;
float ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax;
float Ransac_Z_ROI;
float DBscan_eps;
float DBscan_minPts;
int DB_MinClusterSize, DB_MaxClusterSize;
float EC_eps;
int EC_MinClusterSize, EC_MaxClusterSize;
double ransac_distanceThreshold; //함수 : double형 parameter
bool switch_NoiseFiltering;
bool switch_jiwon_filter;
bool switch_DY_filter;
bool switch_ROI;
//bool switch_DownSampling;
bool switch_Euclid;
bool switch_UpSampling;
bool switch_RanSaC;
bool switch_DBscan;
bool switch_visual;
bool switch_visual_2D;

//cluster information struct using at after clustering
struct objInfo {
    pcl::PointIndices* objPoints;
    float x;
    float y;
    float z;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;
    short intensity;
    string classes;
    unsigned short idx;
};

//func
void ROI(const sensor_msgs::PointCloud2ConstPtr&);
void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
void makeBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
void makeEStopSignal (const PCXYZI&);
void UpSampling(PCXYZI&, PCXYZI::Ptr);
void DownSampling(PCXYZI&, PCXYZI::Ptr);
void NoiseFiltering(PCXYZI::Ptr, PCXYZI::Ptr);
void Clustering (PCXYZI::Ptr, PCXYZI&, bool, bool);
void afterClusteringProcess(PCXYZI::Ptr, PCXYZI&, vector<pcl::PointIndices>&);
void DBScanClustering(PCXYZI::Ptr, PCXYZI&);
void EuclideanClustering(PCXYZI::Ptr, PCXYZI&);
void RanSaC(PCXYZI::Ptr);
void normalEstimation (PCXYZI::Ptr cloud);
string send_msg_DXY(PXYZI);
string send_msg_minmax(float, float, float, float);
string send_msg_cnt(int);
void msg_process(vector<pair<PXYZI,string>>&);
void object_msg_process();

class Filter{
public:
    void DY_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag);
    void DY_filter(vector<objInfo>& objs, bool flag);
    void jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag);
    void jiwon_filter(vector<objInfo>& objs, bool flag);
    inline bool check_in(PXYZI a, PXYZI b) { return ((abs(a.x - b.x) <= REMOVE_FACTOR) && (abs(a.y - b.y) <= REMOVE_FACTOR)); }
    void generate_return_PointCloud(PCXYZI::Ptr inputCloud, PCXYZI& returnCloud, vector<objInfo>& objs);
};
Filter FT;

class Fps{
public:
    Fps();
    void update();
private:
    double prev_clock;
    double cur_clock;
    double interval;
    double m_fps;
    size_t m_count;
};

class RT{
public:
    RT();
    static void start();
    static void end_cal(const char*);
private:
    static double prev_clock;
    static double cur_clock;
    static double interval;
};
double RT::prev_clock;
double RT::cur_clock;
double RT::interval;

inline float cal_dist(float x, float y){ return sqrt(x*x+y*y); }
inline float MidPt(float a, float b){ return (a + b) / 2; }
inline void print_coord(PXYZI tmp){ cout << fixed << setprecision(3) <<"dist : " << cal_dist(tmp.x,tmp.y) << "    x : "<<tmp.x << "    y : "<< tmp.y <<endl; }
inline void print_OBJ(vector<pair<PXYZI,string>>& sorted_OBJ){ for(int i = 0; i < sorted_OBJ.size(); i++) print_coord(sorted_OBJ[i].first); }

template<typename T> //this func is used all code
void pub_process(T& input, sensor_msgs::PointCloud2& output){
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(input, tmp_PCL);                     //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
}

#endif