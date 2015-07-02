#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <drc_task_common/ObstacleDetectionParamsConfig.h>
#include <cmath>
#include <deque>

class ObstacleDetection{
private:
  ros::NodeHandle n;
  ros::Subscriber kdtree_sub;
  ros::Publisher kdtree_pub;
  ros::Publisher marker_pub;
  ros::Publisher stop_real_robot_pub;
  bool brake_flag;
  struct pointsXYZ {
    std::deque<float> x;
    std::deque<float> y;
    std::deque<float> z;
    float ave_x;
    float ave_y;
    float ave_z;
  } points;
  dynamic_reconfigure::Server<drc_task_common::ObstacleDetectionParamsConfig> server;
  dynamic_reconfigure::Server<drc_task_common::ObstacleDetectionParamsConfig>::CallbackType f;
  bool working;
  double front_margin;
  double side_margin;
  double escape_length;
  
public:
  ObstacleDetection(){
    ROS_INFO("START SUBSCRIBING");
    kdtree_sub = n.subscribe("points", 1, &ObstacleDetection::obstacle_detection_cb, this);
    marker_pub = n.advertise<visualization_msgs::Marker>("obstacle_visualization_marker", 1000);
    stop_real_robot_pub = n.advertise<std_msgs::Bool>("stop_real_robot_cmd", 1000);
    brake_flag = true;
    for (int i=0; i < 10; i++){
      points.x.push_back(0.0);
      points.y.push_back(0.0);
      points.z.push_back(10.0);
    }
    f = boost::bind(&ObstacleDetection::dynamic_reconfigure_cb, this, _1, _2);
    server.setCallback(f);

  }
  ~ObstacleDetection(){
  }
  
  /* dynamic_reconfigure for parameter tuning */
  void dynamic_reconfigure_cb(drc_task_common::ObstacleDetectionParamsConfig &config, uint32_t level) {
    // set working
    working = config.obstacle_detection;
    ROS_INFO("obstacle_detection = %s", config.obstacle_detection?"True":"False");

    // set front_margin
    front_margin = config.front_margin;
    ROS_INFO("front_margin = %f", config.front_margin);

    // set side_margin
    side_margin = config.side_margin;
    ROS_INFO("side_margin = %f", config.side_margin);

    // set escape_length
    escape_length = config.escape_length;
    ROS_INFO("escape_length = %f\n\n\n\n\n", config.escape_length);

  }


  void obstacle_detection_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    ROS_INFO("obstacle_detection driven");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Obstacle Detection working?
    if(!working){
      ROS_INFO("Obstacle Detection is not working. if you want to use it change parameters through dynamic reconfigure.\n");
      brake_flag = true;
      return;
    }
    
    // KdTree
    if(cloud->points.size() < 10){
      ROS_INFO("The number of points is too small.\n");
      return;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // create search point at the origin 
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.0;
    searchPoint.y = 0.0;
    searchPoint.z = 0.0;
    
    // K nearest neighbor search
    int K = 1; //3;
    
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    ROS_INFO("K nearest neighbor search at (%f %f %f) with K=%d\n", searchPoint.x, searchPoint.y, searchPoint.z, K);
    
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
      for (size_t i=0; i < pointIdxKNNSearch.size(); i++) {
        ROS_INFO(" %f %f %f (distance: %f)", cloud->points[pointIdxKNNSearch[i]].x, cloud->points[pointIdxKNNSearch[i]].y, cloud->points[pointIdxKNNSearch[i]].z, std::sqrt(pointKNNSquaredDistance[i]) );
      }
    }

    // change queue point data
    points.x.pop_front();
    points.x.push_back(cloud->points[pointIdxKNNSearch[0]].x);

    points.y.pop_front();
    points.y.push_back(cloud->points[pointIdxKNNSearch[0]].y);

    points.z.pop_front();
    points.z.push_back(cloud->points[pointIdxKNNSearch[0]].z);

    // calculate center of 10-frame nearest points
    points.ave_x = points.ave_y = points.ave_z = 0.0;
    for (std::deque<float>::iterator it = points.x.begin(); it != points.x.end(); it++) {
      points.ave_x += *it;
    }
    points.ave_x /= 10;
    for (std::deque<float>::iterator it = points.y.begin(); it != points.y.end(); it++) {
      points.ave_y += *it;
    }
    points.ave_y /= 10;
    for (std::deque<float>::iterator it = points.z.begin(); it != points.z.end(); it++) {
      points.ave_z += *it;
    }
    points.ave_z /= 10;

    double distance_xy = points.ave_x*points.ave_x + points.ave_y*points.ave_y;
    //double distance_xyz = points.ave_x*points.ave_x + points.ave_y*points.ave_y + points.ave_z*points.ave_z;
    distance_xy = std::sqrt(distance_xy);
    //distance_xyz = std::sqrt(distance_xyz);

    ROS_INFO("(%f %f) Dintance=%f", points.ave_x, points.ave_y, distance_xy);

    // Visualization Marker for the point closest to searchPoint
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    
    marker.header=msg->header;
    marker.ns = "nearest_points";
    marker.id = 0;
    
    geometry_msgs::Point p;
    p.x = searchPoint.x;
    p.y = searchPoint.y;
    p.z = searchPoint.z;
    marker.points.push_back(p);
    p.x = points.ave_x;
    p.y = points.ave_y;
    p.z = points.ave_z;
    marker.points.push_back(p);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;

    double KNNdistance = std::sqrt(pointKNNSquaredDistance[0]);

    // ang is calculated by using TF between head and car_center
    double ang = std::atan2((points.ave_y-searchPoint.y), (points.ave_x-searchPoint.x));
    double r_front = 1.5 / cos(ang) + front_margin;
    double r_side = 0.9  / sin(ang) + side_margin;

    // PI/3 is the border between front and side
    if (std::fabs(ang) < M_PI/6){
      ROS_INFO("FRONT!!\n");
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;

      // publish topic to stop drc_vehicle_xp900 and for real robot to step on brake pedal
      if (brake_flag && KNNdistance < r_front){
        std_msgs::Bool stop_cmd_msg;
        stop_cmd_msg.data = true;
        stop_real_robot_pub.publish(stop_cmd_msg);

        // change marker color into red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        brake_flag = false;
      }
      if (!brake_flag) {
        // change marker color into red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        if ( KNNdistance > (r_front+escape_length)) {
          brake_flag = true;
        }
      }
    } else {
      ROS_INFO("SIDE!!\n");
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;

      // publish topic for stoping drc_vehicle_xp900
      if (brake_flag && KNNdistance < r_side){
        std_msgs::Bool stop_cmd_msg;
        stop_cmd_msg.data = true;
        stop_real_robot_pub.publish(stop_cmd_msg);

        // change marker color into red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        brake_flag = false;
      }
      if (!brake_flag) {
        // change marker color into red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        if ( KNNdistance > (r_side+escape_length)) {
          brake_flag = true;
        }
      }
    }
    
    marker_pub.publish(marker);
  }
};



int main(int argc, char **argv){
  ros::init(argc, argv, "kdtree_obstacle_detection");
  ROS_INFO("START OBSTACLE DETECTION");
  ObstacleDetection KdTree;
  ros::spin();
  return 0;
}
