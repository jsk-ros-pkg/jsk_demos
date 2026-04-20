/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_srvs/Empty.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber slave_joint_states_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;
  ros::ServiceServer set_teaching_mode_server_;
  ros::ServiceServer set_playing_mode_server_;

  // ROS Action Server
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_server_;
  bool follow_action_initialized_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  sensor_msgs::JointState joint_state_msg_;
  std::vector<WayPoint> pre_goal_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;

  double wheel_separation_;
  double wheel_radius_;

  JointTrajectory *jnt_tra_;
  trajectory_msgs::JointTrajectory *jnt_tra_msg_;
  sensor_msgs::JointState slave_joint_states_;

  double read_period_;
  double write_period_;
  double pub_period_;

  bool is_moving_;
  bool is_teaching_;

  std::map<std::string, uint32_t> teaching_torque_;
  std::map<std::string, uint32_t> playing_torque_;
  std::map<std::string, uint32_t> teaching_current_thre_;

 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool initTeachingPlayback(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod(){return read_period_;}
  double getWritePeriod(){return write_period_;}
  double getPublishPeriod(){return pub_period_;}

  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent&);
  void writeCallback(const ros::TimerEvent&);
  void publishCallback(const ros::TimerEvent&);

  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
  void slaveJointStatesCallback(const sensor_msgs::JointState msg);
  void followJointTrajectoryActionGoalCallback();
  void followJointTrajectoryActionPreemptCallback();
  bool setTeachingModeCallback(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res);
  bool setPlayingModeCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res);
  void setTorqueForTeachingPlayback(std::map<std::string, uint32_t> torque);

  void sendMotionFromYaml(const std::string yaml_file);
  bool getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg);
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
