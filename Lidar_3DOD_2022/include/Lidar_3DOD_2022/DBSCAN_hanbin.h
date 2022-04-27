#ifndef DBSCAN_HANBIN
#define DBSCAN_HANBIN

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

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

using namespace std;

inline bool comparePointClusters_hb (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

class pointInformation{
public:
    int self_idx;
    int ID;
    double eps;
    int minPts;
    bool noise;
    int state;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    pair<double,double> nearDist_x; //min, max
    pair<double,double> nearDist_y;
    pair<double,double> nearDist_z;
    vector<int> near_point;

    pointInformation(){

    }

    pointInformation(int self_idx, double eps, int minPts, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
        this -> self_idx = self_idx;
        this ->ID = 0;
        this -> eps = eps;
        this -> minPts = minPts;
        this -> input_cloud = input_cloud;
        this -> noise = 0;
        this -> state = UN_PROCESSED;
        near_point.push_back(self_idx);
        calc_eps();
        calc_dist();
        get_nearPoint_info();
    }

    void calc_eps(){
        double safe_coeff = 0.05; //오차를 고려한 안전계수
        double cur_dist = sqrt(input_cloud->points[self_idx].x * input_cloud->points[self_idx].x + input_cloud->points[self_idx].y * input_cloud->points[self_idx].y + input_cloud->points[self_idx].z * input_cloud->points[self_idx].z);
        this -> eps = 0.037 * cur_dist; //실험적으로 구한 dist에 따른 eps함수
        if(cur_dist > 5){
            this -> eps += safe_coeff; //가까이에선 거의 오차가 없음. 오히려 기존 eps보다 안전계수가 커져버려서 과부하 발생
        }
    }

    void calc_dist(){
        nearDist_x = make_pair(input_cloud->points[self_idx].x - eps, input_cloud->points[self_idx].x + eps);
        nearDist_y = make_pair(input_cloud->points[self_idx].y - eps, input_cloud->points[self_idx].y + eps);
        nearDist_z = make_pair(input_cloud->points[self_idx].z - eps, input_cloud->points[self_idx].z + eps);
    }

    bool outRange(double x, double y, double z){
        if(nearDist_x.first > x || nearDist_x.second < x) return 1;
        if(nearDist_y.first > y || nearDist_y.second < y) return 1;
        if(nearDist_z.first > z || nearDist_z.second < z) return 1;
        return 0;
    }

    void get_nearPoint_info(){
        for(int i = 0; i < input_cloud -> points.size(); i++){
            if(i == self_idx) continue;
            //if(near_point.size() >= minPts) break;
            if(outRange(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z)) continue;
            near_point.push_back(i);
        }        
    }
};

template <typename PointT>
class DBSCAN {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    PointCloudPtr input_cloud;
    KdTreePtr search_method;
    double eps = 0;
    int minPts = 1; // not including the point itself.
    int min_pts_per_cluster = 1;
    int max_pts_per_cluster = std::numeric_limits<int>::max();
    vector<pointInformation> PCinfo;

    void setInputCloud(PointCloudPtr cloud) {
        input_cloud = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method = tree;
    }
    
    void setClusterTolerance(double tolerance) {
        eps = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts = core_point_min_pts;
    }

    void initializer_pointinfo(){
        for(int i = 0; i < input_cloud -> points.size(); i++){
            pointInformation tmp(i, eps, minPts, input_cloud);
            PCinfo.push_back(tmp);
        }
        //cout << "init point_num : " << PCinfo.size() << endl;
    }

    int get_ID(vector<int>& near_pt){
        int tmp_ID = 0;
        for(int i=0;i<near_pt.size();i++){
            if(PCinfo[near_pt[i].self_idx].ID != 0){
                tmp_ID = PCinfo[near_pt[i].self_idx].ID; //여러개 중복되도 상관없음
            }
        }
        return tmp_ID;
    }
    //같은 ID를 가진 애들을 다시 탐색하면서 바꿔주는건 시간낭비임. 애초에 클러스터링을 하면서 동시에 머지해야함
    //그렇게 메모이제이션처럼 사용해야함.
    void clustering_ID(){//전처리 하면서 동시호출 안됨. 아직 모든 포인트에 대한 pointInformation가 만들어 지지 않아서

    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices){
        initializer_pointinfo(); //n^2
        cout << "-----------------------------------------------" << endl;
        for(int i=0;i<PCinfo.size();i++){
            cout<<i<<"번 째 인덱스 near pt 갯수   :   "<<PCinfo[i].near_point.size()<<endl;
        }
        cout << "-----------------------------------------------" << endl;

        for (int i = 0; i < input_cloud->points.size(); i++){
            if(PCinfo[i].state == PROCESSED || PCinfo[i].noise == 1){ PCinfo[i].state = PROCESSED; continue;}
            std::vector<int> processingQueue;
            processingQueue.push_back(i);
            PCinfo[i].state = PROCESSED;

            for(int j = 1; j < PCinfo[i].near_point.size(); j++){
                if (PCinfo[i].self_idx == j) continue;
                processingQueue.push_back(PCinfo[PCinfo[i].near_point[j]].self_idx);
                PCinfo[PCinfo[i].near_point[j]].state = PROCESSING;
            }

            int search_idx = 0;
            while (search_idx < processingQueue.size()) {
                int cur_idx = processingQueue[search_idx];
                if(PCinfo[cur_idx].state == PROCESSED || PCinfo[i].noise == 1){ PCinfo[cur_idx].state = PROCESSED; search_idx++; continue;}
                if (PCinfo[cur_idx].noise != 1) {
                    for (int j = 1; j < PCinfo[cur_idx].near_point.size(); j++) {
                        if (PCinfo[PCinfo[cur_idx].near_point[j]].state == UN_PROCESSED) {
                            processingQueue.push_back(PCinfo[PCinfo[cur_idx].near_point[j]].self_idx);
                            PCinfo[PCinfo[cur_idx].near_point[j]].state = PROCESSING;
                        }
                    }
                }
                PCinfo[cur_idx].state = PROCESSED;
                search_idx++;
            }
            //cout << "queue size : " << processingQueue.size() << endl;

            if (processingQueue.size() >= min_pts_per_cluster && processingQueue.size () <= max_pts_per_cluster) {
                pcl::PointIndices r;
                r.indices.resize(processingQueue.size());
                for (int j = 0; j < processingQueue.size(); ++j) {
                    r.indices[j] = processingQueue[j];
                }
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        }
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters_hb);
    }

};

#endif // DBSCAN_H