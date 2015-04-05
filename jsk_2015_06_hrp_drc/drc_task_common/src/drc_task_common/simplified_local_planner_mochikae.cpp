#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <drc_task_common/Int8Float64.h>
#include <dynamic_reconfigure/server.h>
#include <drc_task_common/LocalPlannerMochikaeParamsConfig.h>
#include <cmath>
#include <algorithm>
#include <deque>
#include <boost/thread/thread.hpp>


class LocalPlanner{
private:
  ros::NodeHandle n;
  ros::Subscriber point_sub;
  ros::Subscriber steering_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber stepon_gaspedal_sub;
  ros::Subscriber mochikae_minmax_sub;
  // ros::Subscriber gaspedal_state_sub;
  // ros::Subscriber brakepedal_state_sub;
  ros::Publisher point_pub;
  ros::Publisher steering_with_index_pub;
  ros::Publisher steering_pub;
  ros::Publisher mochikae_order_pub;
  // ros::Publisher torso_yaw_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  double steering_state;
  double goal_ang;
  bool stepon_gaspedal;
  // bool gaspedal_flag;
  // bool brakepedal_flag;
  double steering_output_ave;
  std::deque<double> steering_output;
  sensor_msgs::PointCloud2 pub_msg; // for debug
  boost::mutex mutex;
  dynamic_reconfigure::Server<drc_task_common::LocalPlannerMochikaeParamsConfig> server;
  dynamic_reconfigure::Server<drc_task_common::LocalPlannerMochikaeParamsConfig>::CallbackType f;
  double mochikae_max_s;
  double mochikae_min_s;
  std::vector<double> option_s;
  double s_inc;
  bool mochikae_flag;
  int grasp_zero_index;
  double grasp_zero_angle;
  double a;
  double b;
  double play;
  
  double max_steering;
  double mochikae_max_steering;
  double mochikae_min_steering;
  double mochikae_threshold_factor;
  bool empty_flag;
  int path_num;
  int field_of_vision;
  double wheelbase;
  double tread;


  int visualize_path;
  bool use_mochikae;
  double path_margin;
  double empty_factor;
  double steering_output_gain;
  // bool use_sigmoid;
  // double sigmoid_gain;
  // bool use_min_max_in_sqrt;
  double xi;
  double eta;
  double zeta;
  int queue_size;


public:
  LocalPlanner(){
    ROS_INFO("START SUBSCRIBING");
    point_sub = n.subscribe("points", 1, &LocalPlanner::local_planner_cb, this);
    steering_sub = n.subscribe("steering", 1, &LocalPlanner::steering_cb, this);
    goal_sub = n.subscribe("goal_dir", 1, &LocalPlanner::goal_dir_cb, this);
    stepon_gaspedal_sub = n.subscribe("real_robot/stepon_gaspedal", 1, &LocalPlanner::stepon_gaspedal_cb, this);
    mochikae_minmax_sub = n.subscribe("real_robot/mochikae/min_max", 1, &LocalPlanner::mochikae_minmax_cb, this);

    // gaspedal_state_sub = n.subscribe("gaspedal/state", 1, &LocalPlanner::gaspedal_state_cb, this);
    // brakepedal_state_sub = n.subscribe("brakepedal/state", 1, &LocalPlanner::brakepedal_state_cb, this);
    point_pub = n.advertise<sensor_msgs::PointCloud2>("curve_path/points2", 1);
    steering_with_index_pub = n.advertise<drc_task_common::Int8Float64>("local_planner/steering/index_cmd", 1);
    steering_pub = n.advertise<std_msgs::Float64>("local_planner/steering/cmd", 1);
    mochikae_order_pub = n.advertise<std_msgs::Bool>("real_robot/mochikae/flag", 1);
    // torso_yaw_pub = n.advertise<std_msgs::Float64>("/look_around_angle", 1);
    steering_state = 0.0;
    empty_flag = true;
    steering_output_ave = 0.0;
    n.param("max_steering", max_steering, 2*M_PI); // -360[deg] ~ 360[deg]
    n.param("mochikae_max_steering", mochikae_max_steering, 1.85*M_PI/3.0); // 111[deg]
    n.param("mochikae_min_steering", mochikae_min_steering, -1.85*M_PI/3.0); // -111[deg]
    n.param("mochikae_threshold_factor", mochikae_threshold_factor, 0.05);
    n.param("path_num", path_num, 13);
    n.param("field_of_vision", field_of_vision, 80);
    n.param("wheelbase", wheelbase, 2.05);
    n.param("tread", tread, 1.4);

    
    stepon_gaspedal = false;
    f = boost::bind(&LocalPlanner::dynamic_reconfigure_cb, this, _1, _2);
    server.setCallback(f);
    mochikae_flag = true;

    grasp_zero_angle = 0.0;
    if (path_num > 20) {
      ROS_INFO("path_num is too large. 19 is set in path_num");
      path_num = 19;
    }

    grasp_zero_index = (path_num + 1) / 2;
    steering_output_gain = 1.0;
    
    // temporary
    // mochikae_max_steering = 1.85 * M_PI / 3.0;
    // mochikae_min_steering = - 1.85 * M_PI / 3.0;
    a = 0.0258676;
    play = 0.60952311;
    b = - a * play;
    // b = -0.0157669;
    // play = std::fabs(b/a);
    mochikae_max_s = std::asin(wheelbase * (a*mochikae_max_steering+b));
    mochikae_min_s = std::asin(wheelbase * (a*mochikae_min_steering-b));
    s_inc = (mochikae_max_s - mochikae_min_s) / (double)(path_num - 1);
    
    for (int i=0; i <= path_num; i++) {
      if (i == 0) {
	option_s.push_back(mochikae_max_s);
      }
      option_s.push_back(mochikae_max_s - s_inc * (i-1));
    }
  }
  ~LocalPlanner(){
  }
  
  
  /* dynamic_reconfigure for parameter tuning */
  void dynamic_reconfigure_cb(drc_task_common::LocalPlannerMochikaeParamsConfig &config, uint32_t level) {
    // set visualize_path
    if (config.visualize_path <= path_num) {
      visualize_path = config.visualize_path;
      ROS_INFO("visualize_path = %d", config.visualize_path);
    } else {
      visualize_path = 1;
      ROS_INFO("The number of path is not more than %d, so default(1) is set as visualize_path", config.visualize_path);
    }
    // set use_mochikae
    use_mochikae = config.use_mochikae;
    ROS_INFO("use_mochikae = %s", config.use_mochikae?"True":"False");
    // set path_margin
    path_margin = config.path_margin;
    ROS_INFO("path_margin = %f", config.path_margin);
    // set use_sigmoid
    // use_sigmoid = config.use_sigmoid;
    // ROS_INFO("use_sigmoid = %s", config.use_sigmoid?"True":"False");
    // set sigmoid_gain
    // sigmoid_gain = config.sigmoid_gain;
    // ROS_INFO("sigmoid_gain = %f", config.sigmoid_gain);
    // set use_min_max_in_sqrt
    // use_min_max_in_sqrt = config.use_min_max_in_sqrt;
    // ROS_INFO("use_min_max_in_sqrt = %s", config.use_min_max_in_sqrt?"True":"False");

    // set steering_curvature_slope_gain
    steering_output_gain = config.steering_output_gain;
    ROS_INFO("steering_output_gain = %f", config.steering_output_gain);
    // set empty_factor
    empty_factor = config.empty_factor;
    ROS_INFO("empty_factor = %f", config.empty_factor);
    // set difference factor
    xi = config.difference_factor;
    ROS_INFO("difference_factor = %f", config.difference_factor);
    // set heading_factor
    eta = config.heading_factor;
    ROS_INFO("heading_factor = %f", config.heading_factor);
    // set distance_factor
    zeta = config.distance_factor;
    ROS_INFO("distance_factor = %f", config.distance_factor);
    // set queue_size
    queue_size = config.queue_size;
    ROS_INFO("queue_size = %d\n\n", config.queue_size);
  }
  
  
  /* callback if /hand_wheel/state is subscribed */
  void steering_cb(const std_msgs::Float64ConstPtr& msg){
    steering_state = msg->data;
  }
  
  
  /* callback if /cheat_goal_dir/ang is subscribed */
  void goal_dir_cb(const std_msgs::Float64ConstPtr& msg){
    goal_ang = msg->data;
  }
  
  
  /* callback if /stepon_gaspedal is subscribed */
  void stepon_gaspedal_cb(const std_msgs::BoolConstPtr& msg){
    stepon_gaspedal = msg->data;
  }
  
  
  /* callback if /gas_pedal/state is subscribed */
  // void gaspedal_state_cb(const std_msgs::Float64ConstPtr& msg){
  //   if (msg->data > 0.01) {
  //     gaspedal_flag = true;
  //   } else {
  //     gaspedal_flag = false;
  //   }
  // }
  
  /* callback if /brake_pedal/state is subscribed */
  // void brakepedal_state_cb(const std_msgs::Float64ConstPtr& msg){
  //   if (msg->data > 0.01) {
  //     brakepedal_flag = true;
  //   } else {
  //     brakepedal_flag = false;
  //   }
  // }
  
  
  /* callback if point cloud is subscribed */
  void local_planner_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    mutex.lock();
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pub_msg.header = msg->header;
    mutex.unlock();
  }

  
  /* callback if mochikae is finished */
  void mochikae_minmax_cb(const std_msgs::Float64MultiArrayConstPtr& msg){
    int i = 0;
    for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
      if (i == 0) {
	mochikae_max_steering = *it;
      }
      if (i == 1) {
	mochikae_min_steering = *it;
      }
      i++;
    }
    
    mochikae_flag = false;
  }  

  
  
  /* execute function */
  void execute(){
    ROS_INFO("local_planner_mochikae driven");
    mutex.lock();

    if (check_condition() == -1) {
      mutex.unlock();
      return;
    }
    
    double steering = steering_state;
    std::vector<double> obstacle_length(path_num+1);
    std::vector<double> cost(path_num+1);
    
    
    for (int i=1; i <= path_num; i++) {
      double radius;
      if (option_s[i] != 0){
	radius = std::fabs(1 / s_to_kappa_calculation(option_s[i]));
      } else {
	radius = 10000;
      }

      int sign;
      if (option_s[i] >= 0) {
	sign = 1;
      } else {
	sign = -1;
      }

      pcl::PointXYZ center_point;
      
      center_point.x = - wheelbase / 2.0;
      center_point.y = (double)sign * std::sqrt(radius*radius - wheelbase*wheelbase);
      center_point.z = 0.0;
      
      ROS_INFO("i=%d, s=%f, radius=%f, center_point=(%.3f, %.3f, 0)", i, option_s[i], radius, center_point.x, center_point.y);
      obstacle_length[i] = kdtree_curve(cloud, center_point, std::fabs(radius));

      if (i == visualize_path) {
        if (empty_flag == false) {
          point_pub.publish(pub_msg);
        } else {
          ROS_INFO("\nPath %d has no points, this cannot be published.\n", visualize_path);
        }
      }
    }
    mutex.unlock();
    
    double cost_max = 0;
    int ret_idx = 0;
    
    // if (use_sigmoid == true) {
    //   normalize_sigmoid(obstacle_length);
    // } else {
      normalize_sqrt(obstacle_length);
    // }

    for (int i=1; i <= path_num; i++) {
      ROS_INFO("Path %d", i);
      cost[i] = cost_function(steering_output_ave, option_s[i], goal_ang, obstacle_length[i]);

      if (cost_max < cost[i]) {
        cost_max = cost[i];
        ret_idx = i;
      }
    }

    ROS_INFO("\n\n          Path %d, cost = %lf\n\n", ret_idx, cost_max);

    if (stepon_gaspedal != true) {
      // change queue steering output data
      if (steering_output.size() >= queue_size) {
	steering_output.pop_front();
      }
      //steering_output.push_back( (double)(s_inc*(grasp_zero_index-ret_idx)));
      steering_output.push_back(s_to_alpha_transformation(option_s[ret_idx]));
    } else {
      // send joint trajectory to STARO
      // yaw_flag
    }
    //   if (gaspedal_flag == true) {
    //     // change queue steering output data
    //     if (steering_output.size() >= queue_size) {
    //       steering_output.pop_front();
    //     }
    //     steering_output.push_back( (double)(s_inc*(grasp_zero_index-ret_idx)) );
    //   } else {
    //     if (brakepedal_flag == true) {
    //       double look_around = (steering_output_ave*M_PI/max_steering + goal_ang) / 2;
    //     }
    //   }


    // calculate averaged steering angle for 10-frame
    steering_output_ave = 0.0;
    for (std::deque<double>::iterator it = steering_output.begin(); it != steering_output.end(); it++) {
      steering_output_ave += *it;
    }
    
    if (steering_output.size()==0) {
      return;
    }
    ROS_INFO("deque.size() = %ld", steering_output.size());
    steering_output_ave = (double)steering_output_ave / steering_output.size() * steering_output_gain;
    
    
    drc_task_common::Int8Float64 pub_steering_with_index_msg;
    pub_steering_with_index_msg.index = ret_idx;
    pub_steering_with_index_msg.data = alpha_to_s_transformation(steering_output_ave);
    steering_with_index_pub.publish(pub_steering_with_index_msg);

    std_msgs::Float64 pub_steering_msg;
    pub_steering_msg.data = steering_output_ave;
    steering_pub.publish(pub_steering_msg);


    // for mochikae

    // if (ret_idx == 1 || ret_idx == path_num){
    //   if ( steering_output_ave >= (mochikae_max_steering-mochikae_threshold_factor*std::fabs(mochikae_max_steering)) ) {
    // 	grasp_zero_angle = mochikae_max_steering;
    //   } else if ( steering_output_ave >= (mochikae_min_steering+mochikae_threshold_factor*std::fabs(mochikae_min_steering)) ) {
    // 	grasp_zero_angle = mochikae_min_steering;
    //   }
    //   if ( use_mochikae == true ){
    // 	mochikae_flag = true;
    //   }
    //   std_msgs::Bool mochikae_msg;
    //   mochikae_msg.data = true;
    //   mochikae_order_pub.publish(mochikae_msg);

    // }
  }
  // ---execute()
  
  
  int check_condition(){
    if (!cloud) {
      ROS_INFO("cloud doesn't have resource");
      return -1;
    }
    
    // K-d Tree
    if(cloud->points.size() < 10) {
      ROS_INFO("The number of points is too small.");
      return -1;
    }
    
    if (path_num%2 == 0) {
      ROS_INFO("path_num should not be even number, so default(13) is set as path num");
      path_num = 13;
    }

    if (mochikae_flag == true) {
      ROS_INFO("mochikae is not finished. option steer angle is set.");
      
      mochikae_max_s = alpha_to_s_transformation(mochikae_max_steering);
      mochikae_min_s = alpha_to_s_transformation(mochikae_min_steering);
      s_inc = (mochikae_max_s - mochikae_min_s) / (double)(path_num - 1);

      ROS_INFO("mochikae_max_s = %f, mochikae_min_s = %f, s_inc = %f", mochikae_max_s, mochikae_min_s, s_inc);

      option_s[0] = mochikae_max_s; 
      option_s[1] = mochikae_max_s; 
      for (int i=2; i <= path_num; i++) {
	if (i == grasp_zero_index) {
	  option_s[i] = 0;
	} else {
	  option_s[i] = option_s[i-1] - s_inc;
	}
      }
      if (use_mochikae != true) {
	mochikae_flag = false;
      }
      
      return -1;
    }
    
    return 1;
  }
  
  
  double alpha_to_kappa_table(double alpha){
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

  double kappa_to_alpha_table(double kappa){
    // kappa = a * alpha + b
    double alpha;
    if (kappa > 0){
      alpha = (kappa - b) / a;
    } else if (kappa < 0) {
      alpha = (kappa + b) / a;
    } else {
      alpha = 0;
    }

    return alpha;
  }

  
  double kappa_to_s_calculation(double kappa){
    double s;
    s = std::asin(wheelbase * kappa);

    return s;
  }

  double s_to_kappa_calculation(double s){
    double kappa;
    kappa = std::sin(s) / wheelbase;

    return kappa;
  }


  double s_to_alpha_transformation(double s){
    double alpha;
    alpha = kappa_to_alpha_table(s_to_kappa_calculation(s));

    return alpha;
  }


  double alpha_to_s_transformation(double alpha){
    double s;
    s = kappa_to_s_calculation(alpha_to_kappa_table(alpha));

    return s;
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
      ROS_INFO("curve_temp->points.size() = 0");
      ROS_INFO("nearest_dist is empty path length, which means %lf", empty_path_length);
      empty_flag = true;
      return (empty_factor * empty_path_length);
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
      ROS_INFO("Cannot make kdtree (within range) because there are no points.");
      ROS_INFO("nearest_dist is empty path length, which means %lf", empty_path_length);
      empty_flag = true;
      return (empty_factor * empty_path_length);
    }

    if (curve->points.size() == 0) {
      ROS_INFO("curve->points.size() = 0");
      ROS_INFO("nearest_dist is empty path length, which means %lf", empty_path_length);
      empty_flag = true;
      return (empty_factor * empty_path_length);
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
      ROS_INFO("nearest_dist=%lf", nearest_dist);
    }
    double nearest_arc = calc_chord_to_arc(nearest_dist, r);

    std_msgs::Header tmp_header = pub_msg.header;
    pcl::toROSMsg(*curve, pub_msg);
    pub_msg.header = tmp_header;

    empty_flag = false;
    return nearest_arc;
  }
  
  
  
  /* calculate arc length from chord length and radius */
  double calc_chord_to_arc(double d, double r){
    double theta = d / (2*r);
    double l = 2 * r * std::asin(theta);
    return l;
  }
  
  
  
  /* calculate turning radius difference for each radius */
  double calc_turning_radius_difference(double r){
    double front_r = std::sqrt( wheelbase*wheelbase + (r-tread/2)*(r-tread/2) );
    double back_r = r - tread/2;
    return (front_r - back_r);
  }
  
  
  
  // double normalize_sigmoid(std::vector<double>& obstacle){
  //   for (int i=1; i<= path_num; i++) {
  //     obstacle[i] = 2.0/(1 + std::exp(-sigmoid_gain*obstacle[i])) - 1;
  //   }
  // }
  
  
  
  double normalize_sqrt(std::vector<double>& obstacle){
    double max_length = obstacle[1];
    double min_length = obstacle[1];
    for (int i=1; i <= path_num; i++) {
      if (max_length < obstacle[i]) {
        max_length = obstacle[i];
      }
      if (min_length > obstacle[i]) {
        min_length = obstacle[i];
      }
    }
    for (int i=1; i<= path_num; i++) {
      // if (use_min_max_in_sqrt == true) {
        obstacle[i] = std::sqrt( (obstacle[i]-min_length) / (max_length-min_length) );
      // } else {
      //   obstacle[i] = std::sqrt(obstacle[i]/max_length);
      // }
    }
  }
  
  
  
  /* cost function for calculating each steering */
  double cost_function(double current_steering, double option_s, double goal, double obstacle){
    double option_steering = s_to_alpha_transformation(option_s);

    double difference = ((mochikae_max_steering - mochikae_min_steering) - std::fabs(current_steering - option_steering))/(double)(mochikae_max_steering - mochikae_min_steering);
    double heading = (2 * M_PI - std::fabs(option_s - goal)) / (2 * M_PI);
    // double heading = ((mochikae_max_steering - mochikae_min_steering) - std::fabs(goal/M_PI*max_steering - option_steering))/(double)(mochikae_max_steering - mochikae_min_steering); // -PI < goal < PI
    
    ROS_INFO("difference  = %f, difference * factor = %f", difference, xi*difference);
    ROS_INFO("abs(option_s - goal) = %f, heading  = %f, eta * heading = %f", std::fabs(option_s-goal), heading, eta*heading);
    ROS_INFO("obstacle  = %f, zeta * obstacle = %f", obstacle, zeta*obstacle);
    double cost = xi * difference + eta * heading + zeta * obstacle;
    ROS_INFO("cost=%lf", cost);
    return cost;
  }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "local_planner");
  ROS_INFO("START LOCAL PLANNING");
  LocalPlanner local_planner;
  ros::Rate rate(2);
  while (ros::ok()){
    // ros::Rate rate(local_planner.publish_rate);
    local_planner.execute();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
