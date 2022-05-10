#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <Lidar_3DOD_2022/Lidar_declare.h>

void tracking_process(const sensor_msgs::PointCloud2ConstPtr& aft_ROI){
    RT1.start();

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker
        (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;


    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);


  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence
    (new ApproxNearestPairPointCloudCoherence<RefPointType>);

  DistanceCoherence<RefPointType>::Ptr distance_coherence
    (new DistanceCoherence<RefPointType>);
  coherence->addPointCoherence (distance_coherence);

  pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (0.01));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.01);

  tracker_->setCloudCoherence (coherence);



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

	ros::Subscriber sub = nh.subscribe<Lidar_3DOD_2022::obj_msg> ("/Lidar_obj", 100, tracking_process);
    
	ros::spin();
}               