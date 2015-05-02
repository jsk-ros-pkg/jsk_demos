// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/


#ifndef DRC_TASK_COMMON_STANDING_DRILL_DETECTOR_H_
#define DRC_TASK_COMMON_STANDING_DRILL_DETECTOR_H_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl_ros/pcl_nodelet.h> // Include all the dependency of pcl_ros

#include <jsk_pcl_ros/geo_util.h>
namespace drc_task_common
{
  class StandingDrillDetector
  {
  public:
    typedef boost::shared_ptr<StandingDrillDetector> Ptr;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBoxArray,
      jsk_recognition_msgs::ClusterPointIndices>
    SyncPolicy;
    
    StandingDrillDetector();

  protected:

    virtual void detect(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg);
    
    virtual jsk_pcl_ros::Cylinder::Ptr estimateStandingDrill(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const jsk_recognition_msgs::BoundingBox& box);
    virtual double computeMinimumDistance(
      const pcl::PointXYZRGB& p,
      const std::vector<jsk_pcl_ros::ConvexPolygon::Ptr>& polygons);
    virtual std::vector<jsk_pcl_ros::ConvexPolygon::Ptr> cubeSideFaces(
      const Eigen::Affine3f& pose,
      jsk_pcl_ros::Vertices& local_points);
    virtual void publishPoseStamped(
      ros::Publisher& pub,
      const std_msgs::Header& header,
      Eigen::Affine3f& pose);
    virtual double computeFootCoefficients(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const Eigen::Affine3f pose,
      const jsk_recognition_msgs::BoundingBox& box);
    
    virtual double computeFootCoefficients2(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const Eigen::Affine3f pose);
    ros::NodeHandle nh_, pnh_;
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_debug_cylinder_pose_;
    ros::Publisher pub_debug_foot_pose_;
    ros::Publisher pub_foot_marker_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_box_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;

    
  private:
    
  };
}

#endif
