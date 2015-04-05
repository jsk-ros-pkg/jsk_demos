#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <drc_task_common/VisualOdometryVerificationConfig.h>
#include <cmath>
#include <deque>

class VisualOdometryVerification{
private:
  ros::NodeHandle n;
  ros::Subscriber odom_sub;
  ros::Publisher steering_pub;
  ros::Publisher velocity_pub;
  ros::Publisher ang_vel_pub;
  ros::Publisher stop_flag_pub;
  std::deque<double> linear_velocity_deque;
  std::deque<double> angular_velocity_deque;
  std::deque<double> steering_output_deque;
  double filtered_linear_velocity;
  double filtered_angular_velocity;
  double filtered_steering_angle;
  dynamic_reconfigure::Server<drc_task_common::VisualOdometryVerificationConfig> server;
  dynamic_reconfigure::Server<drc_task_common::VisualOdometryVerificationConfig>::CallbackType f;
  double wheelbase;
  double stop_threshold;
  double wheel_steering_weight;
  bool stop_flag;
  double LPF_vel_weight;
  double LPF_ang_vel_weight;
  bool use_linear_LPF;
  double LPF_steering_weight;
  bool use_moving_average;
  int queue_size;
  double a;
  double b;

  
public:
  VisualOdometryVerification(){
    ROS_INFO("START SUBSCRIBING");
    odom_sub = n.subscribe("odometry", 1, &VisualOdometryVerification::odom_cb, this);
    steering_pub = n.advertise<std_msgs::Float64>("steering", 1000);
    velocity_pub = n.advertise<std_msgs::Float64>("velocity", 1000);
    ang_vel_pub = n.advertise<std_msgs::Float64>("angular_velocity", 1000);
    stop_flag_pub = n.advertise<std_msgs::Bool>("stop_flag", 1000);
    n.param("wheelbase", wheelbase, 2.05);
    n.param("stop_threshold", stop_threshold, 0.1);
    n.param("wheel_steering_weight", wheel_steering_weight, 4.0);
    f = boost::bind(&VisualOdometryVerification::dynamic_reconfigure_cb, this, _1, _2);
    server.setCallback(f);
    stop_flag = true;

    // temporary
    a = 0.0258676;
    b = -0.0157669;
  }
  ~VisualOdometryVerification(){
  }
  
  
  /* dynamic_reconfigure for parameter tuning */
  void dynamic_reconfigure_cb(drc_task_common::VisualOdometryVerificationConfig &config, uint32_t level) {
    // set LPF_vel_weight
    LPF_vel_weight = config.LPF_vel_weight;
    ROS_INFO("LPF_vel_weight = %f", config.LPF_vel_weight);

    // set LPF_ang_vel_weight
    LPF_ang_vel_weight = config.LPF_ang_vel_weight;
    ROS_INFO("LPF_ang_vel_weight = %f", config.LPF_ang_vel_weight);
    // set use_linear_LPF
    use_linear_LPF = config.use_linear_LPF;
    ROS_INFO("use_linear_LPF = %s", config.use_linear_LPF?"True":"False");

    // set LPF_steering_weight
    LPF_steering_weight = config.LPF_steering_weight;
    ROS_INFO("LPF_steering_weight = %f", config.LPF_steering_weight);

    // set use_moving_average
    use_moving_average = config.use_moving_average;
    ROS_INFO("use_moving_average = %s", config.use_moving_average?"True":"False");

    // set queue_size
    queue_size = config.queue_size;
    ROS_INFO("queue_size = %d\n\n\n", config.queue_size);
  }
  
  /* callback if odometry is subscribed */
  void odom_cb(const nav_msgs::OdometryConstPtr& msg){
    double vx, vy, vz, wx, wy, wz;
    double linear_velocity, angular_velocity;

    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    vz = msg->twist.twist.linear.z;
    wx = msg->twist.twist.angular.x;
    wy = msg->twist.twist.angular.y;
    wz = msg->twist.twist.angular.z;

    linear_velocity = std::sqrt(vx*vx + vy*vy);
    angular_velocity = wz;

    // change queue velocity output data
    if (linear_velocity_deque.size() >= 2) {
      linear_velocity_deque.pop_front();
    }
    linear_velocity_deque.push_back(linear_velocity);
    // filtered by linear_LPF
    filtered_linear_velocity = linear_LPF(linear_velocity_deque, LPF_vel_weight);

    // change queue angular velocity output data
    if (angular_velocity_deque.size() >= 2) {
      angular_velocity_deque.pop_front();
    }
    angular_velocity_deque.push_back(angular_velocity);
    // filtered by linear_LPF
    filtered_angular_velocity = linear_LPF(angular_velocity_deque, LPF_ang_vel_weight);
    
    
    // publish linear velocity
    std_msgs::Float64 vel_msg;
    vel_msg.data = filtered_linear_velocity;
    velocity_pub.publish(vel_msg);

    // publish angular velocity
    std_msgs::Float64 ang_vel_msg;
    ang_vel_msg.data = filtered_angular_velocity;
    ang_vel_pub.publish(ang_vel_msg);

    // publish angular velocity
    std_msgs::Float64 stop_flag_msg;

    if (linear_velocity > stop_threshold) {
      stop_flag = false;
      stop_flag_msg.data = stop_flag;
      stop_flag_pub.publish(stop_flag_msg);
      ROS_INFO("visual odometry verification driven");
    } else {
      stop_flag = true;
      stop_flag_msg.data = stop_flag;
      stop_flag_pub.publish(stop_flag_msg);
      ROS_INFO("visual odometry verification not driven (stop_flag == true)");
      return;
    }


    double steering_angle;
    steering_angle = calc_steering_angle_from_twist(linear_velocity, angular_velocity);
    // steering_angle = calc_steering_angle_from_twist(filtered_linear_velocity, filtered_angular_velocity);
    if (std::isfinite(steering_angle)) {
      ROS_INFO("steering_angle = %f", steering_angle);
    } else {
      ROS_INFO("steering_angle is NaN");
      return;
    }
    
    // change queue steering output data
    if (steering_output_deque.size() >= queue_size) {
      steering_output_deque.pop_front();
    }
    steering_output_deque.push_back(steering_angle);
    // ROS_INFO("queue_size = %d", queue_size);

    // linear_LPF
    if (use_linear_LPF == true) {
      filtered_steering_angle = linear_LPF(steering_output_deque, LPF_steering_weight);
      return;
    }

    // moving_average
    if (use_moving_average == true) {
      filtered_steering_angle = moving_average(steering_output_deque);
      return;
    }
    
    // without any filters
    filtered_steering_angle = steering_angle;
  }
  
  
  /* calculate steering angle from twist */
  double calc_steering_angle_from_twist(double v, double w){
    double kappa = w / v; // (= 1/r)
    double alpha;

    // if use drive simulator
    // alpha = std::asin( wheelbase * w / v ) * wheel_steering_weight;

    if (kappa > 0){
      alpha = (kappa - b) / a;
    } else if (kappa < 0) {
      alpha = (kappa + b) / a;
    } else {
      alpha = 0;
    }

    return alpha;
  }
  
  
  
  /* linear LPF */
  double linear_LPF(std::deque<double>& data, double weight){
    double output_lpf;
    if (data.size() >= 2) {
      // ROS_INFO("before data.size()-1 = %ld, data[data.size()-1] = %f", data.size()-1, data[data.size()-1]);
      output_lpf = weight * data[data.size()-1] + (1 - weight) * data[data.size()-2];
    } else {
      output_lpf = data[data.size()-1];
    }
    
    data[data.size()-1] = output_lpf;
    // ROS_INFO("output_lpf = %f", output_lpf);
    // ROS_INFO("after data.size()-1 = %ld, data[data.size()-1] = %f", data.size()-1, data[data.size()-1]);
    return output_lpf;
  }

  
  /* moving average */
  double moving_average(std::deque<double>& data){
    double steering_output_ave = 0.0;
    for (std::deque<double>::iterator it = data.begin(); it != data.end(); it++) {
      steering_output_ave += *it;
    }

    steering_output_ave = (double)steering_output_ave / data.size();

    return steering_output_ave;
  }
  
  
  /* execute */
  void execute(){
    if (stop_flag==false) {
      std_msgs::Float64 steering;
      steering.data = filtered_steering_angle;
      steering_pub.publish(steering);
    }
  }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "visual_odometry_verification");
  ROS_INFO("START VERIFICATION BY VISUAL ODOMETRY");
  VisualOdometryVerification VO_verification;
  ros::Rate rate(10);
  while (ros::ok()){
    VO_verification.execute();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
