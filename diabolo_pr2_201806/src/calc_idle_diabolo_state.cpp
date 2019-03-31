#include <cmath>
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

class CalcIdleDiaboloStateNode
{
public:
  CalcIdleDiaboloStateNode() : nh_(""), pnh_("~"), r_(30),
			       pitch_(0), yaw_(0),  			       
			       min_cube_x_(0.3), max_cube_x_(1.0),
			       min_cube_y_(-0.2), max_cube_y_(0.2),
			       min_cube_z_(0.1), max_cube_z_(0.7)

  {
    // Subscriber
    sub_pointcloud_ = pnh_.subscribe("/tf_transform_cloud/output", 1, &CalcIdleDiaboloStateNode::messageCallback, this);

    // Publisher
    pub_diabolo_state_ = pnh_.advertise<std_msgs::Float64MultiArray>("diabolo_state", 1);
    pub_diabolo_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("diabolo_points", 1);
    pub_pitch_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("pitch_points", 1);
    pub_marker_state_ = pnh_.advertise<visualization_msgs::Marker>("marker_state", 1);
    pub_marker_cube_ = pnh_.advertise<visualization_msgs::Marker>("marker_cube", 1);
    pub_marker_mid_ = pnh_.advertise<visualization_msgs::Marker>("marker_mid", 1);

    // Marker
    initMarker();
  }

private:
  void initMarker() {
    /*
     * marker state
     */
    marker_state_.header.frame_id = "/base_footprint";
    marker_state_.header.stamp = ros::Time::now();
    marker_state_.ns = "diabolo_marker";
    marker_state_.id = 0;
    marker_state_.type = visualization_msgs::Marker::CYLINDER;
    marker_state_.action = visualization_msgs::Marker::ADD;
    marker_state_.scale.x = 0.02;
    marker_state_.scale.y = 0.02;
    marker_state_.scale.z = 0.5;
    marker_state_.color.r = 0.0f;
    marker_state_.color.g = 1.0f;
    marker_state_.color.b = 0.0f;
    marker_state_.color.a = 0.5;
    marker_state_.lifetime = ros::Duration();
    
    /*
     * marker cube
     */
    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.ns = "idle_diabolo_cube_marker";
    marker_cube_.id = 1;
    marker_cube_.type = visualization_msgs::Marker::CUBE;
    marker_cube_.action = visualization_msgs::Marker::ADD;
    marker_cube_.scale.x = 0.01;
    marker_cube_.scale.y = 0.01;
    marker_cube_.scale.z = 0.5;
    marker_cube_.color.r = 1.0f;
    marker_cube_.color.g = 0.2f;
    marker_cube_.color.b = 0.0f;
    marker_cube_.color.a = 0.3;
    marker_cube_.lifetime = ros::Duration();
    
    /*
     * marker mid
     */
    marker_mid_.header.frame_id = "/base_footprint";
    marker_mid_.header.stamp = ros::Time::now();
    marker_mid_.ns = "diabolo_mid_marker";
    marker_mid_.id = 0;
    marker_mid_.type = visualization_msgs::Marker::CUBE;
    marker_mid_.action = visualization_msgs::Marker::ADD;
    marker_mid_.scale.x = 0.01;
    marker_mid_.scale.y = 0.5;
    marker_mid_.scale.z = 0.5;
    marker_mid_.color.r = 0.0f;
    marker_mid_.color.g = 0.0f;
    marker_mid_.color.b = 1.0f;
    marker_mid_.color.a = 1.0;
    marker_mid_.lifetime = ros::Duration();
  }

  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_pointcloud)
  {
    // translate ros msg to pointcloud
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*msg_pointcloud, pointcloud);

    // msgs
    std_msgs::Float64MultiArray msg_diabolo_state;
    pcl::PointCloud<pcl::PointXYZ> diabolo_points, pitch_points;
    sensor_msgs::PointCloud2 msg_diabolo_points, msg_pitch_points;

    /*
     * calculate some value to use calculating pitch, yaw, ...
     */
    double max_diabolo_x = -1000;
    double min_diabolo_x = 1000;
    double sum_diabolo_x = 0, sum_diabolo_y = 0, sum_diabolo_z = 0, sum_diabolo_xy = 0, sum_diabolo_x2 = 0;
    int diabolo_cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = pointcloud.points.begin(); p != pointcloud.points.end(); *p++) {
      if (p->x < max_cube_x_ and p->x > min_cube_x_ and p->y > min_cube_y_ and p->y < max_cube_y_ and p->z > min_cube_z_ and p->z < max_cube_z_) {	
	max_diabolo_x = (max_diabolo_x > p->x) ? max_diabolo_x : p->x;
	min_diabolo_x = (min_diabolo_x < p->x) ? min_diabolo_x : p->x;

	sum_diabolo_x += p->x;
	sum_diabolo_y += p->y;
	sum_diabolo_z += p->z;
	sum_diabolo_xy += p->x * p->y;
	sum_diabolo_x2 += p->x * p->x;
	diabolo_cnt++;

	diabolo_points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
      }
    }
    
    /*
     * publish diabolo points
     */
    pcl::toROSMsg(diabolo_points, msg_diabolo_points);
    msg_diabolo_points.header.frame_id = "/base_footprint";
    msg_diabolo_points.header.stamp = ros::Time::now();
    pub_diabolo_points_.publish(msg_diabolo_points);

    /*
     * calculate pitch points and publish
     */
    double max_z_temae = 0;
    double max_x_temae;
    double max_z_oku = 0;
    double max_x_oku;
    double mid_diabolo_x = (max_diabolo_x + min_diabolo_x) / 2.;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = pointcloud.points.begin(); p != pointcloud.points.end(); *p++) {
      if (p->x < max_cube_x_ and p->x > min_cube_x_ and p->y > min_cube_y_ and p->y < max_cube_y_ and p->z > min_cube_z_ and p->z < max_cube_z_) {		
	if (p->x > mid_diabolo_x) {
	  if (max_z_oku < p->z) {
	    max_z_oku = p->z;
	    max_x_oku = p->x;
	  }
	} else {
	  if (max_z_temae < p->z) {
	    max_z_temae = p->z;
	    max_x_temae = p->x;
	  }
	}
      }
    }
    pitch_points.push_back(pcl::PointXYZ(max_x_oku, 0, max_z_oku));
    pitch_points.push_back(pcl::PointXYZ(max_x_temae, 0, max_z_temae));        
    pcl::toROSMsg(pitch_points, msg_pitch_points);
    msg_pitch_points.header.frame_id = "/base_footprint";
    msg_pitch_points.header.stamp = ros::Time::now();
    pub_pitch_points_.publish(msg_pitch_points);

    /*
     * calculate pitch and publish
     */
    double tmp_pitch = std::atan2(max_z_oku - max_z_temae, max_x_oku - max_x_temae) / 3.14 * 180;
    if (not std::isnan(tmp_pitch)) {
      pitch_ = tmp_pitch;
    }
    msg_diabolo_state.data.push_back(pitch_);

    /*
     * calculate yaw and publish
     */
    double tmp_yaw = std::atan2((diabolo_cnt * sum_diabolo_xy - sum_diabolo_x * sum_diabolo_y), (diabolo_cnt * sum_diabolo_x2 - sum_diabolo_x * sum_diabolo_x)) / 3.14 * 180;
    if (not std::isnan(tmp_yaw)) {
      yaw_ = tmp_yaw;
    }
    msg_diabolo_state.data.push_back(yaw_);
    
    /*
     * publish diabolo_state
     */
    pub_diabolo_state_.publish(msg_diabolo_state);    
    std::cout << "[pitch] " << pitch_ << " [yaw] " << yaw_ << std::endl;

    /*
     * calculate marker state and publish
     */
    marker_state_.header.frame_id = "/base_footprint";
    marker_state_.header.stamp = ros::Time::now();
    marker_state_.pose.position.x = sum_diabolo_x / diabolo_cnt;
    marker_state_.pose.position.y = sum_diabolo_y / diabolo_cnt;
    marker_state_.pose.position.z = sum_diabolo_z / diabolo_cnt;
    tf::Quaternion q_diabolo = tf::createQuaternionFromRPY(0, -pitch_ * 3.14 / 180 + 1.57, yaw_ * 3.14 / 180);
    quaternionTFToMsg(q_diabolo, marker_state_.pose.orientation);
    pub_marker_state_.publish(marker_state_);
    
    /*
     * calculate marker cube and publish
     */
    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.pose.position.x = (max_cube_x_ + min_cube_x_) / 2.0;
    marker_cube_.pose.position.y = (max_cube_y_ + min_cube_y_) / 2.0;
    marker_cube_.pose.position.z = (max_cube_z_ + min_cube_z_) / 2.0;
    marker_cube_.scale.x = std::abs(max_cube_x_ - min_cube_x_);
    marker_cube_.scale.y = std::abs(max_cube_y_ - min_cube_y_);
    marker_cube_.scale.z = std::abs(max_cube_z_ - min_cube_z_);
    pub_marker_cube_.publish(marker_cube_);
    
    /*
     * calculate marker mid and publish
     */
    marker_mid_.header.frame_id = "/base_footprint";
    marker_mid_.header.stamp = ros::Time::now();
    marker_mid_.pose.position.x = mid_diabolo_x;
    marker_mid_.pose.position.y = sum_diabolo_y / diabolo_cnt;
    marker_mid_.pose.position.z = sum_diabolo_z / diabolo_cnt;
    pub_marker_mid_.publish(marker_mid_);
  }

  // ros params
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pointcloud_;
  ros::Publisher pub_diabolo_state_;                                   // Float64MultiArray
  ros::Publisher pub_diabolo_points_, pub_pitch_points_;               // PointCloud
  ros::Publisher pub_marker_state_, pub_marker_cube_, pub_marker_mid_; // Marker
  visualization_msgs::Marker marker_state_, marker_cube_, marker_mid_;  
  ros::Rate r_;

  // cube params
  double min_cube_x_, max_cube_x_;
  double min_cube_y_, max_cube_y_;
  double min_cube_z_, max_cube_z_;

  // diabolo state
  double pitch_, yaw_;  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_idle_diabolo_state");

  CalcIdleDiaboloStateNode n;
  ros::spin();

  return 0;
}
