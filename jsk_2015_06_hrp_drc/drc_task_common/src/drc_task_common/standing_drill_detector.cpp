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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "drc_task_common/standing_drill_detector.h"
#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>

namespace drc_task_common
{
  //const double drill_foot_x = 0.115;
  const double drill_foot_x = 0.125;
  const double drill_foot_y = 0.085;
  const double drill_foot_z = 0.02;
  const double drill_foot_x_offset x= 0.015;
  StandingDrillDetector::StandingDrillDetector():
    pnh_("~")
  {
    pub_marker_ = pnh_.advertise<visualization_msgs::Marker>("cylinder_marker", 1);
    pub_foot_marker_ = pnh_.advertise<visualization_msgs::Marker>("foot_marker", 1);
    pub_debug_cylinder_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/cylinder_pose", 1);
    pub_debug_foot_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/foot_pose", 1);
    sub_cloud_.subscribe(pnh_, "input", 1);
    sub_box_.subscribe(pnh_, "input/box_array", 1);
    sub_indices_.subscribe(pnh_, "input/indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_box_, sub_indices_);
    sync_->registerCallback(boost::bind(&StandingDrillDetector::detect, this, _1, _2, _3));
  }

  void StandingDrillDetector::detect(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    std::vector<pcl::PointIndices::Ptr> indices
      = pcl_conversions::convertToPCLPointIndices(
        indices_msg->cluster_indices);
    
    for (size_t i = 0; i < indices.size(); i++) {
      pcl::PointIndices::Ptr the_indices = indices[i];
      jsk_recognition_msgs::BoundingBox box_msg = box_array_msg->boxes[i];
      pcl::ExtractIndices<pcl::PointXYZRGB> ex;
      ex.setInputCloud(cloud);
      ex.setIndices(the_indices);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      ex.filter(*segment_cloud);
      ROS_INFO("segment_cloud: %lu", segment_cloud->points.size());
      jsk_pcl_ros::Cylinder::Ptr cylinder = estimateStandingDrill(segment_cloud, box_msg);
    }
  }

  jsk_pcl_ros::Cylinder::Ptr StandingDrillDetector::estimateStandingDrill(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const jsk_recognition_msgs::BoundingBox& box)
  {
    ROS_INFO("input cloud to detect cylinder: %lu", cloud->points.size());
    pcl::CropBox<pcl::PointXYZRGB> crop_box(false);
    
    Eigen::Vector4f max_points(box.dimensions.x/2,
                               box.dimensions.y/2,
                               box.dimensions.z/2 - 0.05, // 5cm offset
                               0);
    Eigen::Vector4f min_points(-box.dimensions.x/2,
                               -box.dimensions.y/2,
                               box.dimensions.z/2 - 0.05 - 0.1, // 5cm offset
                               0);
    ROS_INFO("max: [%f, %f, %f, %f]", max_points[0], max_points[1], max_points[2], max_points[3]);
    ROS_INFO("min: [%f, %f, %f, %f]", min_points[0], min_points[1], min_points[2], min_points[3]);
    Eigen::Affine3f pose;
    tf::poseMsgToEigen(box.pose, pose);
    float roll, pitch, yaw;
    pcl::getEulerAngles(pose, roll, pitch, yaw);
    crop_box.setTranslation(pose.translation());
    crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    ROS_INFO("r, p, y: [%f, %f, %f]", roll, pitch, yaw);
    ROS_INFO("pos: [%f, %f, %f]", crop_box.getTranslation()[0], crop_box.getTranslation()[1], crop_box.getTranslation()[2]);
    crop_box.setInputCloud(cloud);
    crop_box.setMax(max_points);
    crop_box.setMin(min_points);
    
    pcl::PointIndices::Ptr cropped_indices (new pcl::PointIndices);
    crop_box.filter(cropped_indices->indices);
    ROS_INFO("cropped points: %lu", cropped_indices->indices.size());
    if (cropped_indices->indices.size() == 0) {
      ROS_FATAL("no enough cropped indices");
      return jsk_pcl_ros::Cylinder::Ptr();
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(cropped_indices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    ex.filter(*cropped_cloud);
    // Estimate normal
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setIndices(cropped_indices);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.02);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMethodType(pcl::SAC_MSAC);
    //seg.setMethodType(pcl::SAC_LMEDS);
    //seg.setMethodType(pcl::SAC_PROSAC);
    seg.setAxis(pose.rotation() * Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(0.2);
    // phi 60 ~ 75
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(1000000);
    seg.setNormalDistanceWeight(0.05);
    seg.setRadiusLimits(0.025, 0.035);
    seg.setProbability(0.8);
    seg.setInputCloud(cropped_cloud);
    seg.setInputNormals(normals);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() > 0) {
      Eigen::Vector3f dir(coefficients->values[3],
                          coefficients->values[4],
                          coefficients->values[5]);
      if (dir.dot(Eigen::Vector3f(0, -1, 0)) < 0) {
        dir = - dir;
      }
      
      jsk_pcl_ros::Cylinder::Ptr cylinder(new jsk_pcl_ros::Cylinder(
                                            Eigen::Vector3f(
                                              coefficients->values[0],
                                              coefficients->values[1],
                                              coefficients->values[2]),
                                            dir,
                                            coefficients->values[6]));
      pcl::PointIndices::Ptr cylinder_indices
        (new pcl::PointIndices);
      pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
      for (size_t i = 0; i < cropped_cloud->points.size(); i++) {
        pcl::PointXYZ p;
        p.x = cropped_cloud->points[i].x;
        p.y = cropped_cloud->points[i].y;
        p.z = cropped_cloud->points[i].z;
        xyz_cloud.points.push_back(p);
      }
      cylinder->filterPointCloud(xyz_cloud,
                                 0.015,
                                 *cylinder_indices);
      double height = 0;
      Eigen::Vector3f center;
      
      cylinder->estimateCenterAndHeight(
        xyz_cloud, *cylinder_indices,
        center, height);
      visualization_msgs::Marker cylinder_marker;
      Eigen::Vector3f support_direction = pose.rotation() * Eigen::Vector3f::UnitZ();
      // dir
      cylinder->toMarker(cylinder_marker, center, support_direction, height);
      cylinder_marker.header = box.header;
      pub_marker_.publish(cylinder_marker);

      Eigen::Affine3f cylinder_pose = Eigen::Translation3f(center) * pose.rotation();
      publishPoseStamped(pub_debug_cylinder_pose_, box.header, cylinder_pose);
      const size_t resolution = 200;
      size_t best_i = 0;
      double best_coef = DBL_MAX;
      Eigen::Affine3f best_pose = Eigen::Affine3f::Identity();
      const double min_angle = 0;
      const double max_angle = M_PI * 2.0;
      // Downsample
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(0.005, 0.005, 0.005);
      vg.filter(*downsampled_cloud);
      
      for (size_t i = 0; i < resolution; i++) {
        //Eigen::Affine3f foot_pose = cylinder_pose * Eigen::Translation3f(Eigen::Vector3f(0, 0, 0.08));
        Eigen::Affine3f foot_pose = cylinder_pose * Eigen::Translation3f(Eigen::Vector3f(0, 0.0, 0.09));
        double theta = min_angle + (max_angle - min_angle) * i / resolution;
        //double theta = i * 2.0 * M_PI / resolution;
        foot_pose = foot_pose * Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
        Eigen::Vector3f x_direction = cylinder_pose.rotation() *  Eigen::Vector3f::UnitX();
        foot_pose = foot_pose * Eigen::Translation3f(- x_direction * drill_foot_x_offset);
        
        double coef = computeFootCoefficients(downsampled_cloud, foot_pose, box);
        if (coef != DBL_MAX) {
          ROS_INFO("coef: [%f] => %f", theta / M_PI * 180, coef);
        }
        if (coef <= best_coef) {
          
          best_coef = coef;
          best_i = 0;
          best_pose = foot_pose;
        }
      }
      ROS_INFO("best_coef: %f", best_coef);
      publishPoseStamped(pub_debug_foot_pose_, box.header, best_pose);
      visualization_msgs::Marker foot_marker;
      foot_marker.header = box.header;
      tf::poseEigenToMsg(best_pose, foot_marker.pose);
      foot_marker.scale.x = drill_foot_x;
      foot_marker.scale.y = drill_foot_y;
      foot_marker.scale.z = drill_foot_z;
      foot_marker.color.r = 1.0;
      foot_marker.color.a = 1.0;
      foot_marker.type = visualization_msgs::Marker::CUBE;
      pub_foot_marker_.publish(foot_marker);
      return cylinder;
    }
    else {
      ROS_ERROR("Failed to detect cylinder");
      return jsk_pcl_ros::Cylinder::Ptr();
    }
  }

  std::vector<jsk_pcl_ros::ConvexPolygon::Ptr> StandingDrillDetector::cubeSideFaces(
    const Eigen::Affine3f& pose,
    jsk_pcl_ros::Vertices& local_points)
  {
    const double x = drill_foot_x / 2.0;
    const double y = drill_foot_y / 2.0;
    const double z = drill_foot_z / 2.0;
    // local points
    Eigen::Vector3f Al( x,  y,  z);
    Eigen::Vector3f Bl(-x,  y,  z);
    Eigen::Vector3f Cl(-x, -y,  z);
    Eigen::Vector3f Dl( x, -y,  z);
    Eigen::Vector3f El( x,  y, -z);
    Eigen::Vector3f Fl(-x,  y, -z);
    Eigen::Vector3f Gl(-x, -y, -z);
    Eigen::Vector3f Hl( x, -y, -z);
    Eigen::Vector3f A = pose * Al;
    Eigen::Vector3f B = pose * Bl;
    Eigen::Vector3f C = pose * Cl;
    Eigen::Vector3f D = pose * Dl;
    Eigen::Vector3f E = pose * El;
    Eigen::Vector3f F = pose * Fl;
    Eigen::Vector3f G = pose * Gl;
    Eigen::Vector3f H = pose * Hl;
    local_points.push_back(A);
    local_points.push_back(B);
    local_points.push_back(C);
    local_points.push_back(D);
    local_points.push_back(E);
    local_points.push_back(F);
    local_points.push_back(G);
    local_points.push_back(H);
    jsk_pcl_ros::Vertices a_vertices, b_vertices, c_vertices, d_vertices;
    a_vertices.push_back(A);
    a_vertices.push_back(B);
    a_vertices.push_back(F);
    a_vertices.push_back(E);
    b_vertices.push_back(B);
    b_vertices.push_back(C);
    b_vertices.push_back(G);
    b_vertices.push_back(F);
    c_vertices.push_back(C);
    c_vertices.push_back(D);
    c_vertices.push_back(H);
    c_vertices.push_back(G);
    d_vertices.push_back(A);
    d_vertices.push_back(D);
    d_vertices.push_back(H);
    d_vertices.push_back(F);
    jsk_pcl_ros::ConvexPolygon::Ptr a_plane(new jsk_pcl_ros::ConvexPolygon(a_vertices));
    jsk_pcl_ros::ConvexPolygon::Ptr b_plane(new jsk_pcl_ros::ConvexPolygon(b_vertices));
    jsk_pcl_ros::ConvexPolygon::Ptr c_plane(new jsk_pcl_ros::ConvexPolygon(c_vertices));
    jsk_pcl_ros::ConvexPolygon::Ptr d_plane(new jsk_pcl_ros::ConvexPolygon(d_vertices));
    std::vector<jsk_pcl_ros::ConvexPolygon::Ptr> planes;
    planes.push_back(a_plane);
    planes.push_back(b_plane);
    planes.push_back(c_plane);
    planes.push_back(d_plane);
    return planes;
  }
  
  double StandingDrillDetector::computeFootCoefficients(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const Eigen::Affine3f pose,
    const jsk_recognition_msgs::BoundingBox& box)
  {
    float roll, pitch, yaw;
    pcl::getEulerAngles(pose, roll, pitch, yaw);
    jsk_pcl_ros::Vertices local_points;
    Eigen::Affine3f box_pose;
    tf::poseMsgToEigen(box.pose, box_pose);
    std::vector<jsk_pcl_ros::ConvexPolygon::Ptr> side_faces = cubeSideFaces(pose, local_points);
    // Check local_points coordinates does not violate bouding box much
    for (size_t i = 0; i < local_points.size(); i++) {
      // Convert local_points to box relative point
      Eigen::Vector3f p = local_points[i];
      Eigen::Vector3f transformed_p = box_pose.inverse() * p;
      
      double width = box.dimensions.x;
      double height = box.dimensions.y;
      
      const double box_offset = 0.05;
      //if (!(transformed_p[1] < 0)) {
      if (false) {
        if (std::abs(transformed_p[0]) > width / 2 + box_offset ||
            std::abs(transformed_p[1]) > height / 2 + box_offset) {
          ROS_ERROR("hypothesis violates bounding box");
          return DBL_MAX;
        }
      }
    }
    pcl::CropBox<pcl::PointXYZRGB> inner_crop_box(false), outer_crop_box(false);
    double offset = 1.0;
    Eigen::Vector4f outer_max_points(drill_foot_x / 2 + offset,
                                     drill_foot_y / 2 + offset,
                                     drill_foot_z / 2,
                                     0);
    Eigen::Vector4f outer_min_points(- drill_foot_x / 2 - offset,
                                     - drill_foot_y / 2 - offset,
                                     - drill_foot_z / 2,
                                     0);
    outer_crop_box.setTranslation(pose.translation());
    outer_crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    
    outer_crop_box.setMax(outer_max_points);
    outer_crop_box.setMin(outer_min_points);

    outer_crop_box.setInputCloud(cloud);
    
    pcl::PointIndices::Ptr outer_indices (new pcl::PointIndices);
    outer_crop_box.filter(outer_indices->indices);
    const size_t min_inlier = 10;
    if (outer_indices->indices.size() < min_inlier) {
      return DBL_MAX;
    }
    double coef = 0;
    for (size_t i = 0; i < outer_indices->indices.size(); i++) {
      pcl::PointXYZRGB p = cloud->points[outer_indices->indices[i]];
      double d = computeMinimumDistance(p, side_faces);
      //coef = d * d * d * d + coef;
      coef = pow(d, 2) + coef;
    }
    return coef;
  }

  double StandingDrillDetector::computeMinimumDistance(
    const pcl::PointXYZRGB& p,
    const std::vector<jsk_pcl_ros::ConvexPolygon::Ptr>& polygons)
  {
    double min_distnce = DBL_MAX;
    Eigen::Vector3f input_point = p.getVector3fMap();
    for (size_t i = 0; i < polygons.size(); i++) {
      Eigen::Vector3f projected_point;
      jsk_pcl_ros::ConvexPolygon::Ptr polygon = polygons[i];
      polygon->projectOnPlane(input_point, projected_point);
      double distance = (projected_point - input_point).norm();
      if (min_distnce > distance) {
        min_distnce = distance;
      }
    }
    return min_distnce;
  }
  
  double StandingDrillDetector::computeFootCoefficients2(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const Eigen::Affine3f pose)
  {
    float roll, pitch, yaw;
    pcl::getEulerAngles(pose, roll, pitch, yaw);
    pcl::CropBox<pcl::PointXYZRGB> inner_crop_box(false), outer_crop_box(false);
    const double offset = 0.005;
    Eigen::Vector4f inner_max_points(drill_foot_x / 2 - offset,
                                     drill_foot_y / 2 - offset,
                                     drill_foot_z / 2 - offset,
                                     0);
    Eigen::Vector4f inner_min_points(- drill_foot_x / 2 + offset,
                                     - drill_foot_y / 2 + offset,
                                     - drill_foot_z / 2 + offset,
                                     0);
    Eigen::Vector4f outer_max_points(drill_foot_x / 2 + offset,
                                     drill_foot_y / 2 + offset,
                                     drill_foot_z / 2 + offset,
                                     0);
    Eigen::Vector4f outer_min_points(- drill_foot_x / 2 - offset,
                                     - drill_foot_y / 2 - offset,
                                     - drill_foot_z / 2 - offset,
                                     0);
    inner_crop_box.setTranslation(pose.translation());
    inner_crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    outer_crop_box.setTranslation(pose.translation());
    outer_crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    
    inner_crop_box.setMax(inner_max_points);
    inner_crop_box.setMin(inner_min_points);
    
    outer_crop_box.setMax(outer_max_points);
    outer_crop_box.setMin(outer_min_points);
    inner_crop_box.setInputCloud(cloud);
    outer_crop_box.setInputCloud(cloud);
    pcl::PointIndices::Ptr inner_indices (new pcl::PointIndices);
    pcl::PointIndices::Ptr outer_indices (new pcl::PointIndices);
    inner_crop_box.filter(inner_indices->indices);
    outer_crop_box.filter(outer_indices->indices);
    int num = std::max((int)outer_indices->indices.size() - (int)inner_indices->indices.size(), 0);
    return num;
  }

  void StandingDrillDetector::publishPoseStamped(
    ros::Publisher& pub,
    const std_msgs::Header& header,
    Eigen::Affine3f& pose)
  {
    geometry_msgs::PoseStamped ros_pose;
    ros_pose.header = header;
    tf::poseEigenToMsg(pose, ros_pose.pose);
    pub.publish(ros_pose);
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "standing_drill_detector");
  drc_task_common::StandingDrillDetector detector;
  ros::spin();
}
