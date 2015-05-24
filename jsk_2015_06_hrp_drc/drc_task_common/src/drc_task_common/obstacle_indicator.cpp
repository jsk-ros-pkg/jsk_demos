#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <drc_task_common/ObstacleIndicatorParamsConfig.h>
#include <cmath>
#include <algorithm>
#include <deque>
#include <boost/thread/thread.hpp>

class ObstacleIndicator{
private:
  ros::NodeHandle n;
  ros::Subscriber obstacle_points_sub;
  ros::Subscriber steering_angle_sub;
  ros::Publisher indicator_pub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  boost::mutex mutex;
  dynamic_reconfigure::Server<drc_task_common::ObstacleIndicatorParamsConfig> server;
  dynamic_reconfigure::Server<drc_task_common::ObstacleIndicatorParamsConfig>::CallbackType f;
  double steering_angle;
  bool empty_flag;

  double a;
  double b;
  double play;
  int field_of_vision;
  double wheelbase;
  double tread;

  double path_margin;

public:
  ObstacleIndicator(){
    obstacle_points_sub = n.subscribe("obstacle", 1, &ObstacleIndicator::obstacle_points_cb, this);
    steering_angle_sub = n.subscribe("steering_angle", 1, &ObstacleIndicator::steering_angle_cb, this);
    indicator_pub = n.advertise<std_msgs::Float32>("obstacle_length/indicator", 1);
    n.param("field_of_vision", field_of_vision, 80);
    n.param("wheelbase", wheelbase, 2.05);
    n.param("tread", tread, 1.4);
    f = boost::bind(&ObstacleIndicator::dynamic_reconfigure_cb, this, _1, _2);
    server.setCallback(f);
    steering_angle = 0;
    empty_flag = false;
    a = 0.031139;
    play = 0.261799;
    b = - a * play;
  }
  ~ObstacleIndicator(){
  }

  /* dynamic_reconfigure for parameter tuning */
  void dynamic_reconfigure_cb(drc_task_common::ObstacleIndicatorParamsConfig &config, uint32_t level) {
    // set path_margin
    path_margin = config.path_margin;
  }

  /* callback if point cloud is subscribed */
  void obstacle_points_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    mutex.lock();
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    mutex.unlock();
  }

  /* callback if steering angle is subscribed */
  void steering_angle_cb(const std_msgs::Float32ConstPtr& msg){
    steering_angle = msg->data * M_PI / 180.0;
  }

  /* execute function */
  void execute(){
    ROS_INFO("obstacle_indicator driven");
    mutex.lock();
    if (check_condition() == -1) {
      mutex.unlock();
      return;
    }

    double obstacle_length;
    double kappa = alpha2kappa_table(steering_angle);
    double radius;
    int sign;
    if (kappa != 0) {
      sign = kappa / std::fabs(kappa);
      radius = 1.0 / kappa;
    } else {
      sign = 1;
      radius = 10000;
    }

    pcl::PointXYZ center_point;
    center_point.x = - wheelbase / 2.0;
    center_point.y = (double)sign * std::sqrt(radius*radius - wheelbase*wheelbase);
    center_point.z = 0.4;

    obstacle_length = kdtree_curve(cloud, center_point, std::fabs(radius));
    mutex.unlock();

    ROS_INFO("OBSTACLE_INDICATOR %.3f", obstacle_length);

    std_msgs::Float32 pub_msg;
    pub_msg.data = obstacle_length;
    indicator_pub.publish(pub_msg);
  }

  int check_condition(){
    if (!cloud) {
      ROS_INFO("cloud doesn't have resource");
      return -1;
    }
    if(cloud->points.size() < 10) {
      ROS_INFO("The number of points is too small.");
      return -1;
    }
    return 1;
  }

  double alpha2kappa_table(double alpha){
    // kappa = a * alpha + b (alpha > 0), kappa = a*alpha - b (alpha < 0)
    double kappa;
    if ( std::fabs(alpha) <= play ){
      kappa = 0;
    } else if ( alpha > play) {
      kappa = a * alpha + b;
    } else {
      kappa = a * alpha - b;
    }
    return kappa;
  }

  /* search curve point and return arc length */
  double kdtree_curve(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  pcl::PointXYZ center, double r){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_out_of_range;
    kdtree_out_of_range.setInputCloud(cloud);

    double empty_path_length = r * field_of_vision * M_PI / 180;

    // remove points outside curve
    std::vector<int> OutOfRange_Index;
    std::vector<float> OutOfRange_SqDist;
    pcl::PointCloud<pcl::PointXYZ>::Ptr curve_temp(new pcl::PointCloud<pcl::PointXYZ>);

    double inner_radius;
    double outer_radius;
    if (r < 1000) {
      inner_radius = r - tread/2 - path_margin - calc_turning_radius_difference(r);
      outer_radius = r + tread/2 + path_margin;
    } else {
      inner_radius = r - tread/2 - path_margin - 0.25;
      outer_radius = r + tread/2 + path_margin + 0.25;
    }

    if (kdtree_out_of_range.radiusSearch(center, inner_radius, OutOfRange_Index, OutOfRange_SqDist) > 0) {
      std::sort(OutOfRange_Index.begin(), OutOfRange_Index.end());
      long idx_max = cloud->points.size();
      for (long i=0, count=0; i < idx_max; i++) {
        if (i != (long)OutOfRange_Index[count]) {
          curve_temp->points.push_back(cloud->points[i]);
        } else {
          count++;
        }
      }
      curve_temp->height = 1;
      curve_temp->width = curve_temp->points.size();
    } else {
      long idx_max = cloud->points.size();
      for (long i=0; i < idx_max; i++) {
        curve_temp->points.push_back(cloud->points[i]);
      }
      curve_temp->height = 1;
      curve_temp->width = curve_temp->points.size();
    }

    if (curve_temp->points.size() == 0) {
      empty_flag = true;
      return empty_path_length;
    }

    // remove points inside curve
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_within_range;
    kdtree_within_range.setInputCloud(curve_temp);

    std::vector<int> WithinRange_Index;
    std::vector<float> WithinRange_SqDist;
    pcl::PointCloud<pcl::PointXYZ>::Ptr curve(new pcl::PointCloud<pcl::PointXYZ>);

    if (kdtree_within_range.radiusSearch(center, outer_radius, WithinRange_Index, WithinRange_SqDist) > 0) {
      std::sort(WithinRange_Index.begin(), WithinRange_Index.end());
      long idx_max = WithinRange_Index.size() - 1;
      for (long i=WithinRange_Index[0], count=0; i < WithinRange_Index[idx_max]; i++) {
        if (i == WithinRange_Index[count]) {
          curve->points.push_back(curve_temp->points[i]);
          count++;
        }
      }
      curve->height = 1;
      curve->width = curve->points.size();
    } else {
      empty_flag = true;
      return empty_path_length;
    }

    if (curve->points.size() == 0) {
      empty_flag = true;
      return empty_path_length;
    }

    double nearest_dist; // distance of nearest point
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_nearest_dist;
    kdtree_nearest_dist.setInputCloud(curve);

    std::vector<int> KNN_Index(1);
    std::vector<float> KNN_SqDist(1);

    pcl::PointXYZ car_front;
    car_front.x = wheelbase / 2.0;
    car_front.y = 0.0;
    car_front.z = 0.0;

    if (kdtree_nearest_dist.nearestKSearch(car_front, 1, KNN_Index, KNN_SqDist) > 0) {
      nearest_dist = std::sqrt(KNN_SqDist[0]);
    }
    double nearest_arc = calc_chord2arc(nearest_dist, r);

    empty_flag = false;
    return nearest_arc;
  }

  /* calculate arc length from chord length and radius */
  double calc_chord2arc(double d, double r){
    double sine = d / (2*r);
    double l = 2 * r * std::asin(sine);
    return l;
  }

  /* calculate turning radius difference for each radius */
  double calc_turning_radius_difference(double r){
    double front_r = std::sqrt( wheelbase*wheelbase + (r-tread/2)*(r-tread/2) );
    double back_r = r - tread/2;
    return (front_r - back_r);
  }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "obstacle_indicator");
  ROS_INFO("START OBSTACLE INDICATOR");
  ObstacleIndicator obstacle_indicator;
  ros::Rate rate(2.5);
  while (ros::ok()){
    obstacle_indicator.execute();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
