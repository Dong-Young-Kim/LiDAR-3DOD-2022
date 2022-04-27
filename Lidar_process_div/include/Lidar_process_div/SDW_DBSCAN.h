#ifndef SDW_DBSCAN_H
#define SDW_DBSCAN_H

#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

using namespace std;

inline bool comparePointClusters_hb (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

class PointInfo{ 
    template<typename PointT> friend class DBSCANclu;
public:
    int self_idx;
    int ID;
    double eps;
    int minPts;
    bool noise;
    int state;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    vector<int> near_point; //indices로 near point 인덱스들 넣기로 수정
    vector<float> sqr_dist; //near point 거리
    void setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr tree) {search_method = tree;}  

    PointInfo(){};
    PointInfo(int self_idx_, double eps_, int minPts_, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_, pcl::search::KdTree<pcl::PointXYZI>::Ptr tree)
    :self_idx(self_idx_), ID(0), eps(eps_), minPts(minPts_), input_cloud(input_cloud_), noise(0), state(UN_PROCESSED), search_method(tree){
        near_point.push_back(self_idx);
        calc_eps();
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

    void get_nearPoint_info(){
        search_method->radiusSearch(self_idx, eps, near_point, sqr_dist);
    }
};


template <typename PointT>
class DBSCANclu {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

    KdTreePtr search_method;
    PointCloudPtr input_cloud;
    double eps = 0;
    int minPts = 1; // not including the point itself.
    int min_pts_per_cluster = 1;
    int max_pts_per_cluster = std::numeric_limits<int>::max();
    vector<PointInfo> PCinfo;

    void setInputCloud(PointCloudPtr cloud) {input_cloud = cloud;}
    void setSearchMethod(KdTreePtr tree) {search_method = tree;}    
    void setClusterTolerance(double tolerance) {eps = tolerance;}
    void setMinClusterSize (int min_cluster_size) {min_pts_per_cluster = min_cluster_size;}
    void setMaxClusterSize (int max_cluster_size) {max_pts_per_cluster = max_cluster_size;}    
    void setCorePointMinPts(int core_point_min_pts) {minPts = core_point_min_pts;}
    void initializer_pointinfo(){
        for(int i = 0; i < input_cloud -> points.size(); i++){
            PointInfo tmp(i, eps, minPts, input_cloud, search_method);
            PCinfo.push_back(tmp);
        }
    }
    void extract(std::vector<pcl::PointIndices>& cluster_indices){
        initializer_pointinfo(); //nlogn
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


#endif //smpg
