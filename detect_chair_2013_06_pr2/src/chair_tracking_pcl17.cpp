/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

using namespace pcl::tracking;

template <typename PointType>
class OpenNISegmentTracking
{
public:
  typedef pcl::PointXYZ RefPointType;
  typedef ParticleXYR ParticleT;

  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  OpenNISegmentTracking ( int thread_nr, double downsampling_grid_size,
                          bool use_fixed/*, const CloudConstPtr input_model_point_cloud*/)
    : new_cloud_ (false)
    , ne_ (thread_nr)
    , counter_ (0)
    , downsampling_grid_size_ (downsampling_grid_size)
      //     	, input_model_point_cloud_ (input_model_point_cloud)
  {
    KdTreePtr tree (new KdTree (false));
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
			
    input_model_point_cloud_.reset(new Cloud());


    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    if (use_fixed)
      {
        boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
          (new ParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
        tracker_ = tracker;
      }
    else
      {
        boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
          (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
        tracker->setMaximumParticleNum (500);
        tracker->setDelta (0.99);
        tracker->setEpsilon (0.2);
        ParticleT bin_size;
        bin_size.x = 0.1f;
        bin_size.y = 0.1f;
        bin_size.roll = 0.1f;
        tracker->setBinSize (bin_size);
        tracker_ = tracker;
      }
    
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    
    tracker_->setParticleNum (400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    // setup coherences
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);
    
    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
  }

 
  void filterPassThrough (const RefCloudConstPtr &cloud, Cloud &result)
  {
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
  }

  
  void gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
  }
  
  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
  }
  
  void tracking (const RefCloudConstPtr &cloud)
  {
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
  }

  void removeZeroPoints (const CloudConstPtr &cloud,
                         Cloud &result)
  {
    for (size_t i = 0; i < cloud->points.size (); i++)
      {
        PointType point = cloud->points[i];
        if (!(fabs(point.x) < 0.01 &&
              fabs(point.y) < 0.01 &&
              fabs(point.z) < 0.01) &&
            !pcl_isnan(point.x) &&
            !pcl_isnan(point.y) &&
            !pcl_isnan(point.z))
          result.points.push_back(point);
      }

    result.width = static_cast<uint32_t> (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }
  
  
  void
  cloud_cb (const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 point_cloud2;
    RefCloudPtr raw_cloud(new RefCloud);

    projector_.transformLaserScanToPointCloud("base_laser_link", *scan, point_cloud2, tfListener_);
    pcl::fromROSMsg<PointType> (point_cloud2, *raw_cloud);


    boost::mutex::scoped_lock lock (mtx_);
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    filterPassThrough (raw_cloud, *cloud_pass_);
    if (counter_ < 10)
      {
        gridSample (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      }
    else if (counter_ == 10)
      {
        //gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
        cloud_pass_downsampled_ = cloud_pass_;
        CloudPtr target_cloud;

        if (target_cloud != NULL)
          {

            RefCloudPtr nonzero_ref (new RefCloud);
            removeZeroPoints (input_model_point_cloud_, *nonzero_ref);

            //					PCL_INFO ("calculating cog\n");

            Eigen::Vector4f c;
            RefCloudPtr transed_ref (new RefCloud);
            pcl::compute3DCentroid<RefPointType> (*nonzero_ref, c);
            Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<RefPointType> (*nonzero_ref, *transed_ref, trans.inverse());
            CloudPtr transed_ref_downsampled (new Cloud);
            gridSample (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
            tracker_->setReferenceCloud (transed_ref_downsampled);
            tracker_->setTrans (trans);	
            tracker_->setMinIndices (int (input_model_point_cloud_->points.size ()) / 2);
          }
        else
          {
            //					PCL_WARN ("euclidean segmentation failed\n");
          }
      }
    else
      {

        gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
        tracking (cloud_pass_downsampled_);
      }
    
    new_cloud_ = true;
    counter_++;
  }
      
  void
  run ()
  {
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/Shadow_filtered2", 5, &OpenNISegmentTracking::cloud_cb , this);

    ros::spin();
	  
  }
  
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr input_model_point_cloud_;
  boost::mutex mtx_;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  boost::shared_ptr<ParticleFilter> tracker_;
  int counter_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
  double downsampling_grid_size_;
  ros::NodeHandle n;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;
};

void
usage (char** argv)
{
  std::cout << "usage: " << argv[0] << " [-C] [-g]\n\n";
}

int
main (int argc, char** argv)
{

  ros::init(argc, argv, "chair_pcl_tracking");
  bool use_fixed = false;

  double downsampling_grid_size = 0.01;
  
  pcl::console::parse_argument (argc, argv, "-d", downsampling_grid_size);
  if (argc < 2)
    {
      usage (argv);
      exit (1);
    }
  
  /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_model_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  */

  // open kinect
  OpenNISegmentTracking<pcl::PointXYZ> v (8, downsampling_grid_size,use_fixed/*,input_model_point_cloud*/);
  v.run ();
}
