#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <dynamic_reconfigure/server.h>
// #include <drc_task_common/PassthroughCarBodyConfig.h>
#include <cmath>

class PassthroughCarBody{
private:
  ros::NodeHandle n;
  ros::Subscriber point_sub;
  ros::Subscriber execute_flag_sub;
  ros::Publisher point_pub;
  double steering_state;
  bool execute_flag;
  // dynamic_reconfigure::Server<drc_task_common::PassthroughCarBodyConfig> server;
  // dynamic_reconfigure::Server<drc_task_common::PassthroughCarBodyConfig>::CallbackType f;

  double min_x;
  double max_x;
  double min_y;
  double max_y;

public:
  PassthroughCarBody(){
    ROS_INFO("START SUBSCRIBING");
    point_sub = n.subscribe("input_points2", 1, &PassthroughCarBody::passthrough_cb, this);
    execute_flag_sub = n.subscribe("execute_flag", 1, &PassthroughCarBody::execute_flag_cb, this);
    point_pub = n.advertise<sensor_msgs::PointCloud2>("passthrough_output/points2", 1);
    execute_flag = false;
    min_x = 1.8;
    max_x = 30.0;
    min_y = -1.2;
    max_y = 1.2;
    // f = boost::bind(&PassthroughCarBody::dynamic_reconfigure_cb, this, _1, _2);
    // server.setCallback(f);
  }
  ~PassthroughCarBody(){
  }
  
  
  
  
  
  /* dynamic_reconfigure for parameter tuning */
  // void dynamic_reconfigure_cb(drc_task_common::PassthroughCarBodyConfig &config, uint32_t level) {
  //   // set min_x
  //   min_x = config.min_x;
  //   ROS_INFO("min_x = %f", config.min_x);

  //   // set max_x
  //   max_x = config.max_x;
  //   ROS_INFO("max_x = %f", config.max_x);

  //   // set min_y
  //   min_y = config.min_y;
  //   ROS_INFO("min_y = %f", config.min_y);

  //   // set max_y
  //   max_y = config.max_y;
  //   ROS_INFO("max_y = %f\n\n\n", config.max_y);
  // }
  
  
  
  
  
  
  /* callback if execute flag is subscribed */
  void execute_flag_cb(const std_msgs::BoolConstPtr& msg){
    execute_flag = msg->data;
  }

  /* callback if point cloud is subscribed */
  void passthrough_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    if (execute_flag == true) {
      std_msgs::Header header = msg->header;
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_body (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back_without_body (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*msg, *cloud);
    
      // ROS_INFO("cloud->points.size() = %ld", cloud->points.size());
      pcl::PassThrough<pcl::PointXYZRGB> pass_x;
      pass_x.setInputCloud(cloud);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(min_x, max_x);
      pass_x.filter(*cloud_without_body);
      
      pass_x.setFilterLimits(-3, min_x);
      pass_x.filter(*cloud_back);
      // ROS_INFO("cloud_front->points.size() = %ld", cloud_without_body->points.size());
      // ROS_INFO("cloud_back->points.size() = %ld", cloud_back->points.size());
      
      pcl::PassThrough<pcl::PointXYZRGB> pass_y;
      pass_y.setInputCloud(cloud_back);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(min_y, max_y);
      pass_y.setFilterLimitsNegative(true);
      pass_y.filter(*cloud_back_without_body);
      
      
      // ROS_INFO("cloud_back_without_body->points.size() = %ld", cloud_back_without_body->points.size());
      
      
      for (long i=0; i < cloud_back_without_body->points.size(); i++) {
        cloud_without_body->points.push_back(cloud_back_without_body->points[i]);
      }
      // ROS_INFO("cloud_without_body->points.size() = %ld\n\n\n", cloud_without_body->points.size());
      cloud_without_body->height = 1;
      cloud_without_body->width = cloud_without_body->points.size();
      
      sensor_msgs::PointCloud2 pub_msg;
      pcl::toROSMsg(*cloud_without_body, pub_msg);
      pub_msg.header = header;
      point_pub.publish(pub_msg);
    } else {
      return;
    }
  }
};
  
  

int main(int argc, char **argv){
  ros::init(argc, argv, "passthrough_car_body");
  ROS_INFO("START PASSTHROUGH CAR BODY");
  PassthroughCarBody passthrough;
  ros::spin();
  return 0;
}
