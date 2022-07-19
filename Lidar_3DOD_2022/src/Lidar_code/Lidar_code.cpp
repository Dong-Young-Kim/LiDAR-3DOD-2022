#include <Lidar_3DOD_2022/Lidar_declare.h>


using namespace std;

void ROI(const sensor_msgs::PointCloud2ConstPtr& scan){
    RT1.start();
    PCXYZI rawData;
    PCXYZI cropedData;
    pcl::fromROSMsg(*scan,rawData);
    if(switch_ROI) makeCropBox(rawData, ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax);
    // if(switch_ROI){
    //     for(int i=0; i < rawData.points.size(); i++){
    //         if(rawData.points[i].x > ROI_xMax || rawData.points[i].x < ROI_xMin
    //         || rawData.points[i].y > ROI_yMax || rawData.points[i].y < ROI_yMin
    //         || rawData.points[i].z > ROI_zMax || rawData.points[i].z < ROI_zMin) continue;
    //         cropedData.push_back(rawData.points[i]);
    //     }
    // }

    sensor_msgs::PointCloud2 output;                        //to output ROIdata formed PC2
    pub_process(rawData,output);
    pub_ROI.publish(output);
    RT1.end_cal("ROI");
}

void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    pcl::CropBox<PXYZI> boxfilter;
    boxfilter.setMin(Eigen::Vector4f(xMin, yMin, zMin, NULL));
    boxfilter.setMax(Eigen::Vector4f(xMax, yMax, zMax, NULL));
    boxfilter.setInputCloud(Cloud.makeShared());
    boxfilter.filter(Cloud);
}

void makeBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    pcl::PassThrough<PXYZI> filter;
    filter.setInputCloud (Cloud.makeShared());
    filter.setFilterFieldName ("x");
    filter.setFilterLimits (xMin, xMax);
    filter.setFilterFieldName ("y");
    filter.setFilterLimits (yMin, yMax);
    filter.setFilterFieldName ("z");
    filter.setFilterLimits (zMin, zMax);
    filter.filter (Cloud);
}

void makeEStopSignal (const PCXYZI& cloud){        
    for(int i=0; i < cloud.points.size(); i++){
        if(cloud.points[i].x > 0.2f && cloud.points[i].x < 0.7f
        && cloud.points[i].y > -0.25f && cloud.points[i].y < 0.25f
        && cloud.points[i].z > -0.7f && cloud.points[i].z < 0.7f) {
            //시그널 만드는 코드 추가
            break;
        }
    }    
}

void UpSampling(PCXYZI& TotalCloud, PCXYZI::Ptr upsampledCloud){
    PCXYZI Data_for_voxel;
    pcl::MovingLeastSquares<PXYZI, PXYZI> filter;
    pcl::search::KdTree<PXYZI>::Ptr kdtree;
    
    copyPointCloud(TotalCloud, Data_for_voxel);
    filter.setInputCloud(Data_for_voxel.makeShared());
    filter.setSearchMethod(kdtree);
    //filter.setComputeNormals (true);
    filter.setSearchRadius(0.03);       // Use all neighbors in a radius of 3cm.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<PXYZI, PXYZI>::SAMPLE_LOCAL_PLANE);
    filter.setUpsamplingRadius(0.025);   // Radius around each point, where the local plane will be sampled.
    filter.setUpsamplingStepSize(0.02); // Sampling step size. Bigger values will yield less (if any) new points.
    filter.process(*upsampledCloud);

    //normalEstimation(upsampledCloud);

    //cout << "PointCloud after upsampling has: " << upsampledCloud->points.size ()  << " data points." << endl; 
    
    //sensor_msgs::PointCloud2 output; 		          
    //pub_process(*upsampledCloud, output);             
    //pub_US.publish(output);
}

void DownSampling(PCXYZI& rawData, PCXYZI::Ptr downsampledCloud){ //Voxelization = DownSampling 
    PCXYZI Data_for_voxel;
    pcl::VoxelGrid<PXYZI> vg;                            //declare voxel

    copyPointCloud(rawData, Data_for_voxel);            //rawData  ->  Data_for_voxel  ... just copy for modify
    vg.setInputCloud (Data_for_voxel.makeShared());     //Data_for_voxel  ->  vg space  ... deep copy , makeShared() return PC
    vg.setLeafSize (voxel_size_x, voxel_size_y, voxel_size_z);		        //voxel size setting(x,y,z)
    vg.filter (*downsampledCloud);                      //voxelized datas are included in downsampledCloud
    //cout << "PointCloud after downsampling has: " << downsampledCloud->points.size ()  << " data points." << endl; 

    //sensor_msgs::PointCloud2 output;
    //pub_process(*downsampledCloud, output);             
    //pub_DS.publish(output);   
}

void NoiseFiltering(PCXYZI::Ptr inputCloud, PCXYZI::Ptr outputCloud){
    pcl::StatisticalOutlierRemoval<PXYZI> tmp;
    tmp.setInputCloud(inputCloud);
    tmp.setMeanK(50);
    tmp.setStddevMulThresh(1.0);
    tmp.filter(*outputCloud);

    //sensor_msgs::PointCloud2 output;
    //pub_process(*outputCloud, output);
    //if(flag == "pre") pub_NF1.publish(output);
    //else if(flag == "post") pub_NF2.publish(output);
}

void Clustering (PCXYZI::Ptr inputCloud, PCXYZI& retCloud, bool switch_DBscan, bool switch_Euclid){
    if( switch_DBscan ) DBScanClustering( inputCloud, retCloud); //prior DBSCAN
    else if( switch_Euclid ) EuclideanClustering( inputCloud, retCloud );
    else retCloud = *inputCloud; //doesn't process clustering

    sensor_msgs::PointCloud2 output; 
    pub_process(retCloud,output); 
    pub_Clu.publish(output); 
}

void afterClusteringProcess(PCXYZI::Ptr inputCloud, PCXYZI& retCloud, vector<pcl::PointIndices>& cluster_indices){
    int cluSz = cluster_indices.size();
    vector<float> obj_x(cluSz); vector<float> obj_y(cluSz); vector<float> obj_z(cluSz);
    vector<float> obj_xMin(cluSz); vector<float> obj_yMin(cluSz); vector<float> obj_zMin(cluSz);
    vector<float> obj_xMax(cluSz); vector<float> obj_yMax(cluSz); vector<float> obj_zMax(cluSz);

    vector<pair<PXYZI,string>> sorted_OBJ; //여기에 minmax가 포함되지 않아서 발생한 문제이므로 이를 포함하는 struct를 만들자
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
        string classes;
        unsigned int idx;
    };
    vector<struct objInfo> objs;

    int j = 0;
    for (vector<pcl::PointIndices>::iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++){

        pair<float,float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); //first = min, second = max
        pair<float,float> y(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        pair<float,float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());

    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PXYZI pt = inputCloud->points[*pit];
            pt.intensity = j % 10;
            retCloud.push_back(pt);
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y ;
            if(pt.y > y.second)     y.second = pt.y; 
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;
    	}

        PXYZI* tmp = new PXYZI();
        tmp->x = MidPt(x.first,x.second); tmp->y = MidPt(y.first,y.second); tmp->z = z.first; //z = min
        pair<PXYZI,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
        sorted_OBJ.push_back(temp);

        objInfo tmp_obj = {&(*it), MidPt(x.first,x.second), MidPt(y.first,y.second), MidPt(z.first,z.second),
                            x.first, y.first, z.first, x.second, y.second, z.second};
        objs.push_back(tmp_obj);

        obj_x[j] = tmp->x; obj_y[j] = tmp->y; obj_z[j] = MidPt(z.first,z.second);
        obj_xMin[j] = x.first; obj_yMin[j] = y.first; obj_zMin[j] = z.first;
        obj_xMax[j] = x.second; obj_yMax[j] = y.second; obj_zMax[j] = z.second;
    }

    cout << "sorted obj size" << sorted_OBJ.size() << endl;
    //cout << "------------------ DF & JF ------------------" << endl;
    FT.DY_filter(sorted_OBJ, switch_DY_filter);
    FT.jiwon_filter(sorted_OBJ, switch_jiwon_filter);
    //print_OBJ(sorted_OBJ);
    msg_process(sorted_OBJ);
    cout << "sorted obj size" << sorted_OBJ.size() << endl;


    {//메시지 발행으로 임시로 넣어놓은 코드
        Lidar_3DOD_2022::obj_msg msg;
        msg.objc = cluSz;
        msg.x = obj_x; msg.y = obj_y; msg.z = obj_z;
        msg.xMin = obj_xMin; msg.yMin = obj_yMin; msg.zMin = obj_zMin;
        msg.xMax = obj_xMax; msg.yMax = obj_yMax; msg.zMax = obj_zMax;

        pub_obj.publish(msg);
    }
    //object_msg_process
    {
        Lidar_3DOD_2022::object_msg_arr msg;
        Lidar_3DOD_2022::object_msg msgCpnt;

        vector<Lidar_3DOD_2022::object_msg> msgConvertVector;
        msg.objc = objs.size();
        for (objInfo obj : objs){
            msgCpnt.x = obj.x;
            msgCpnt.y = obj.y;
            msgCpnt.z = obj.z;
            msgCpnt.xMin = obj.xMin;
            msgCpnt.yMin = obj.yMin;
            msgCpnt.zMin = obj.zMin;
            msgCpnt.xMax = obj.xMax;
            msgCpnt.yMax = obj.yMax;
            msgCpnt.zMax = obj.zMax;
            msgCpnt.classes = "unkwown";
            msgCpnt.idx = 0;
            msgConvertVector.push_back(msgCpnt);
        }
        msg.object_msg_arr = msgConvertVector;
        pub_object.publish(msg);
    }
}

void EuclideanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree

    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    pcl::EuclideanClusterExtraction<PXYZI> ec;           // clustering with Euclidean method
    ec.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    ec.setClusterTolerance(EC_eps); 	                 // dist between points ..  cur : 30cm
    ec.setMinClusterSize(EC_MinClusterSize);		     // minSize the number of point for clustering
    ec.setMaxClusterSize(EC_MaxClusterSize);	         // minSize the number of point for clustering
    ec.setSearchMethod(tree);				             // searching method : tree 
    ec.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    //cout << "Number of clusters is equal to " << cluster_indices.size() << endl;    //return num of clusteringObj
    afterClusteringProcess(inputCloud, retCloud, cluster_indices);
}

void DBScanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree
    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    
    DBSCAN<PXYZI> db;
    db.setCorePointMinPts(DBscan_minPts);                // minimum points of cluster judge
    db.setClusterTolerance(DBscan_eps);                  // dist between points
    db.setMinClusterSize(DB_MinClusterSize);		     // minSize the number of point for clustering
    db.setMaxClusterSize(DB_MaxClusterSize);	         // maxSize the number of point for clustering
    db.setSearchMethod(tree);				             // searching method : tree
    db.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    db.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    //cout << "Number of clusters is equal to " << cluster_indices.size() << endl;    //return num of clusteringObj
    afterClusteringProcess(inputCloud, retCloud, cluster_indices);
}

void RanSaC(PCXYZI::Ptr inputCloud){
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    PCXYZI::Ptr inlierPoints (new PCXYZI ());
    PCXYZI::Ptr inlierPoints_neg (new PCXYZI ());
    pcl::SACSegmentation<PXYZI> seg;
    pcl::ExtractIndices<PXYZI> extract;

    PCXYZI::Ptr lowPoints (new PCXYZI ());
    PCXYZI::Ptr highPoints (new PCXYZI ());
    pcl::PassThrough<PXYZI> filter;
    filter.setInputCloud (inputCloud);
    filter.setFilterFieldName ("z");
    filter.setFilterLimits (-10, -0.2);
    filter.filter (*lowPoints);
    filter.setFilterLimits (-0.2, 10);
    filter.filter (*highPoints);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (ransac_distanceThreshold);
    seg.setMaxIterations (1000);               //최대 실행 수
    seg.setInputCloud(lowPoints); 
    seg.segment (*inliers, coefficients);

    pcl::copyPointCloud<PXYZI>(*lowPoints, *inliers, *inlierPoints);
    extract.setInputCloud (lowPoints);
    extract.setIndices (inliers);
    extract.setNegative (true);     //false
    extract.filter (*inlierPoints_neg);

    //pcl::concatenateFields(*lowPoints, *highPoints, *inlierPoints_neg); //분할 한 두 포인트 병합
    *inlierPoints_neg += *highPoints;

    //pcl::copyPointCloud(inlierPoints_neg, *inputCloud);
    sensor_msgs::PointCloud2 output; 
    pub_process(*inlierPoints_neg, output); 
    pub_RS.publish(output); 
    pub_process(*inlierPoints, output); 
    pub_GND.publish(output); 
}

void normalEstimation (PCXYZI::Ptr cloud)
{  
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(cloud, normals);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
}
