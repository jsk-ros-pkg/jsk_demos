#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <dynamic_reconfigure/server.h>
#include <drc_task_common/SwitchInputCloudConfig.h>

class SwitchInputCloud{
private:
  ros::NodeHandle n;
  ros::Subscriber passthrough_sub;
  ros::Subscriber raw_sub;
  ros::Publisher point_pub;
  sensor_msgs::PointCloud2 pub_msg;
  dynamic_reconfigure::Server<drc_task_common::SwitchInputCloudConfig> server;
  dynamic_reconfigure::Server<drc_task_common::SwitchInputCloudConfig>::CallbackType f;
  bool debug_flag;



public:
  SwitchInputCloud(){
    ROS_INFO("START SUBSCRIBING");
    passthrough_sub = n.subscribe("passthrough/points", 1, &SwitchInputCloud::passthrough_cb, this);
    raw_sub = n.subscribe("raw/points", 1, &SwitchInputCloud::raw_cb, this);
    point_pub = n.advertise<sensor_msgs::PointCloud2>("kdtree_curve/input", 1);
    f = boost::bind(&SwitchInputCloud::dynamic_reconfigure_cb, this, _1, _2);
    server.setCallback(f);
  }
  ~SwitchInputCloud(){
  }
  
  
  /* dynamic_reconfigure */
  void dynamic_reconfigure_cb(drc_task_common::SwitchInputCloudConfig &config, uint32_t level) {
    // set debug
    debug_flag = config.debug_flag;
    ROS_INFO("debug_flag = %s", config.debug_flag?"True":"False");
  }
  
  /* callback if passthrough point cloud is subscribed */
  void passthrough_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    if (debug_flag==false) {
      point_pub.publish(*msg);
    }
  }

  /* callback if raw point cloud is subscribed */
  void raw_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    if (debug_flag==true) {
      point_pub.publish(*msg);
    }
  }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "switch_input_cloud");
  ROS_INFO("START SWITCH INPUT CLOUD");
  SwitchInputCloud switch_cloud;
  ros::spin();
  return 0;
}
