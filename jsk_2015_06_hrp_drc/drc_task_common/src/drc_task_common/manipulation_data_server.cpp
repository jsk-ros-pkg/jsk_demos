#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <drc_task_common/InteractiveMarkerArray.h>
#include <drc_task_common/ICPService.h>
#include <drc_task_common/TMarkerInfo.h>
#include <drc_task_common/GetIKArmPose.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_interactive_marker/MarkerDimensions.h>
#include <jsk_interactive_marker/GetTransformableMarkerPose.h>
#include <jsk_interactive_marker/GetMarkerDimensions.h>
#include <jsk_interactive_marker/GetType.h>
#include <std_msgs/Empty.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <jsk_recognition_msgs/PointsArray.h>
#include <jsk_recognition_msgs/ICPAlignWithBox.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <eigen_conversions/eigen_msg.h>
#include <map>
#include <string>
#include <cstdio>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/serialization.h>
#include <pcl/io/pcd_io.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>
#include <drc_task_common/manipulation_data_helpers.h>
#include <jsk_recognition_msgs/ICPResult.h>

using namespace std;
using namespace ros;
using namespace visualization_msgs;
using namespace manip_helpers;

// input cloud may should be same with icp and box
// for drc, all msg in odom_on_ground may be convenient

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c) {return sqrt(a * a + b * b + c * c);}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}


class ManipulationDataServer
{
public:
  tf::TransformBroadcaster _br;
  typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
						     jsk_recognition_msgs::BoundingBox > SyncPolicy;
  typedef message_filters::sync_policies::ExactTime< jsk_recognition_msgs::BoundingBox,
																										 jsk_recognition_msgs::ICPResult> SyncPolicy_b_i;
  tf::TransformListener _listener;
protected:
  boost::mutex _mutex;
  ros::NodeHandle _node;
  message_filters::Subscriber <sensor_msgs::PointCloud2> _subPoints;
  message_filters::Subscriber <jsk_recognition_msgs::BoundingBox> _subBox;
  message_filters::Subscriber <jsk_recognition_msgs::ICPResult> _subICP;
  Subscriber _subSelectedPoints;
  Subscriber _subSelectedPose;
  Subscriber _sub_pose_feedback;
  Subscriber _sub_object_pose_update;
  Subscriber _sub_object_pose_feedback;
  Subscriber _sub_grasp_pose_feedback;
  Subscriber _sub_grasp_pose_feedback_not_allow_slip;
  Subscriber _sub_grasp_pose_dual_feedback;
  Subscriber _sub_clouds;
  Publisher _pointsPub;
  Publisher _pointsArrayPub;
  Publisher _debug_cloud_pub;
  Publisher _grasp_pose_pub;
  Publisher _grasp_pose_dual_pub;
  Publisher _grasp_pose_dual_z_free_pub;
  Publisher _grasp_pose_z_free_pub;
  Publisher _push_pose_pub;
  Publisher _push_pose_with_assist_pub;
  Publisher _push_pose_with_assist_many_times_pub;
  Publisher _move_by_axial_restraint_pose_pub;
  Publisher _move_by_axial_restraint_not_allow_slip_pose_pub;
  Publisher _reset_pose_pub;
  Publisher _debug_grasp_pub;
  Publisher _reset_pub;
  Publisher _move_pose_pub;
  Publisher _move_pose_dual_pub;
  Publisher _feedback_pub;
  Publisher _box_pub;
  Publisher _debug_point_pub;
  Publisher _marker_set_pose_pub;
  Publisher _t_marker_set_pose_pub;
  Publisher _t_marker_box_pub;
  Publisher _t_marker_information_pub;
  ServiceClient _tf_publish_client;
  ServiceClient _icp_client;
  ServiceClient _get_type_client;
  ServiceClient _get_pose_client;
  ServiceClient _get_dim_client;
  ServiceServer _align_icp_server;
  ServiceServer _save_server;
  ServiceServer _assoc_server;
  ServiceServer _disassoc_server;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >_sync;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy_b_i> >_sync_b_i;
  tf::Transform _tf_from_base;
  tf::Transform _tf_marker;
  tf::Transform _tf_from_camera;
  tf::Transform _tf_object_constraint;
  ros::Timer _tf_timer;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
  interactive_markers::MenuHandler _menu_handler_first;
  interactive_markers::MenuHandler _menu_handler_grasp;
  interactive_markers::MenuHandler _menu_handler_push;
  interactive_markers::MenuHandler _menu_handler_axial_restraint;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _reference_cloud;
  pcl::PointCloud<pcl::Normal>::Ptr _reference_cloud_normals;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _reference_kd_tree;
  geometry_msgs::Vector3 _reference_box_size;
  int _reference_num;
  geometry_msgs::Pose _menu_pose;
  geometry_msgs::Pose _marker_pose;
  geometry_msgs::Pose _grasp_pose;
  std::string _base_link_name;
  // marker 
  std::vector<boost::shared_ptr<ManipulationData> > _manip_data_array;
  boost::shared_ptr<ManipulationData> _manip_data_ptr;
  // now reference marker's ptr ;
  int _timer_count;
  bool _reference_hit;
  bool _assoc_marker_flug;
  bool _all_manual;
  //bool init_reference_end;
public:
  bool transform_pointcloud_in_frame(
    tf::Transform transform,
    const sensor_msgs::PointCloud2& cloud_msg,
    pcl::PointCloud<pcl::PointXYZRGB>& output,
    Eigen::Affine3f& offset,
    tf::TransformListener& tf_listener)
  {
    // convert the pose into eigen
    Eigen::Affine3d box_pose_respected_to_cloud_eigend;
    tf::poseMsgToEigen(tf_to_pose(transform),
                       box_pose_respected_to_cloud_eigend);
    Eigen::Affine3d box_pose_respected_to_cloud_eigend_inversed
      = box_pose_respected_to_cloud_eigend.inverse();
    Eigen::Matrix4f box_pose_respected_to_cloud_eigen_inversed_matrixf;
    Eigen::Matrix4d box_pose_respected_to_cloud_eigen_inversed_matrixd
      = box_pose_respected_to_cloud_eigend_inversed.matrix();
    jsk_pcl_ros::convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(
      box_pose_respected_to_cloud_eigen_inversed_matrixd,
      box_pose_respected_to_cloud_eigen_inversed_matrixf);
    offset = Eigen::Affine3f(box_pose_respected_to_cloud_eigen_inversed_matrixf);
    pcl::PointCloud<pcl::PointXYZRGB> input;
    pcl::fromROSMsg(cloud_msg, input);
    pcl::transformPointCloud(input, output, offset);
  }

  
  ManipulationDataServer():_timer_count(100),_tf_timer(_node.createTimer(ros::Duration(0.1), &ManipulationDataServer::timerCallback, this)), _reference_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), _reference_cloud_normals(new pcl::PointCloud<pcl::Normal>), _reference_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
  {
    //init_reference_end=false;
    init_reference();
    _assoc_marker_flug=false;
    _tf_from_base.setOrigin(tf::Vector3(0, 0, 0));
    _tf_from_base.setRotation(tf::Quaternion(0, 0, 0, 1));
    _tf_marker =_tf_from_base;
    ROS_INFO("start interface");
    ros::NodeHandle local_nh("~");
    local_nh.param("BASE_FRAME_ID", _base_link_name, std::string("/camera_link"));
    local_nh.param("ALL_MANUAL", _all_manual, false);
    ROS_INFO("base_link_name: %s", _base_link_name.c_str());
    _server.reset(new interactive_markers::InteractiveMarkerServer("manip","",false) );
    _menu_handler_first.insert("Grasp", boost::bind(&ManipulationDataServer::grasp_cb, this, _1));
    _menu_handler_first.insert("Grasp_Assist", boost::bind(&ManipulationDataServer::grasp_cb_2, this, _1));
    _menu_handler_first.insert("Push", boost::bind(&ManipulationDataServer::push_cb, this, _1));
    _menu_handler_first.insert("Axial Restraint", boost::bind(&ManipulationDataServer::axial_cb, this, _1));
    _menu_handler_first.insert("Remove", boost::bind(&ManipulationDataServer::remove_cb, this, _1));
    _menu_handler_first.insert("Set Pose With Tracking", boost::bind(&ManipulationDataServer::set_pose_cb, this, _1));
    _menu_handler_first.insert("Reset Pose", boost::bind(&ManipulationDataServer::reset_pose_cb, this, _1));
    _menu_handler_grasp.insert("do_grasp", boost::bind(&ManipulationDataServer::do_grasp_cb, this, _1));
    _menu_handler_grasp.insert("do_grasp_z_free", boost::bind(&ManipulationDataServer::do_grasp_z_free_cb, this, _1));   
    _menu_handler_grasp.insert("Remove", boost::bind(&ManipulationDataServer::remove_cb, this, _1));
    _menu_handler_grasp.insert("Reset Pose", boost::bind(&ManipulationDataServer::reset_pose_cb, this, _1));
    _menu_handler_grasp.insert("Set Pose With Tracking", boost::bind(&ManipulationDataServer::set_pose_cb, this, _1));
    _menu_handler_grasp.insert("Set Grasp Pose", boost::bind(&ManipulationDataServer::set_grasp_pose_cb, this, _1));
    _menu_handler_grasp.insert("Reverse Hand", boost::bind(&ManipulationDataServer::reverse_hand_menu_cb, this, _1));
    _menu_handler_grasp.insert("Move", boost::bind(&ManipulationDataServer::move_hand_menu_cb, this, _1));
    _menu_handler_grasp.insert("Revise Model", boost::bind(&ManipulationDataServer::revise_model_cb, this, _1));
    _menu_handler_push.insert("do_push", boost::bind(&ManipulationDataServer::do_push_cb, this, _1));
    _menu_handler_push.insert("do_push(with assist)", boost::bind(&ManipulationDataServer::do_push_with_assist_cb, this, _1));
    _menu_handler_push.insert("do_push(with assist, many times)", boost::bind(&ManipulationDataServer::do_push_with_assist_many_times_cb, this, _1));
    _menu_handler_push.insert("Remove", boost::bind(&ManipulationDataServer::remove_cb, this, _1));
    _menu_handler_push.insert("Reset Pose", boost::bind(&ManipulationDataServer::reset_pose_cb, this, _1));
    _menu_handler_axial_restraint.insert("move (by axial_restraint)", boost::bind(&ManipulationDataServer::do_move_by_axial_restraion_cb, this, _1));
    _menu_handler_axial_restraint.insert("move (by axial_restraint, not allow slip)", boost::bind(&ManipulationDataServer::do_move_by_axial_restraion_not_allow_slip_cb, this, _1));
    _menu_handler_axial_restraint.insert("Remove", boost::bind(&ManipulationDataServer::remove_cb, this, _1));

    _pointsPub = _node.advertise<sensor_msgs::PointCloud2>("/manip_points", 10);
    _pointsArrayPub = _node.advertise<sensor_msgs::PointCloud2>("/icp_registration/input_reference_add", 10);
    _debug_cloud_pub = _node.advertise<sensor_msgs::PointCloud2>("/manip/debug_cloud", 10);
    _grasp_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/grasp_pose", 10);
    _grasp_pose_dual_pub = _node.advertise<geometry_msgs::PoseArray>("/grasp_dual_pose", 10);
    _grasp_pose_dual_z_free_pub = _node.advertise<geometry_msgs::PoseArray>("/grasp_dual_pose_z_free", 10);
    _grasp_pose_z_free_pub = _node.advertise<geometry_msgs::PoseStamped>("/grasp_pose_z_free", 10);
    _push_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/push_pose", 10);
    _push_pose_with_assist_pub = _node.advertise<geometry_msgs::PoseArray>("/push_pose_with_assist", 10);
    _push_pose_with_assist_many_times_pub = _node.advertise<geometry_msgs::PoseArray>("/push_pose_with_assist_many_times", 10);
    _move_by_axial_restraint_pose_pub = _node.advertise<geometry_msgs::PoseArray>("/move_by_axial_restraint_pose", 10);
    _move_by_axial_restraint_not_allow_slip_pose_pub = _node.advertise<geometry_msgs::PoseArray>("/move_by_axial_restraint_not_allow_slip_pose", 10);
    _debug_point_pub = _node.advertise<geometry_msgs::PointStamped>("/debug_point", 10);
    _reset_pose_pub = _node.advertise<std_msgs::String>("/reset_pose_command", 1);
    _reset_pub = _node.advertise<jsk_recognition_msgs::PointsArray>("/icp_registration/input_reference_array", 1, boost::bind( &ManipulationDataServer::icp_connection, this, _1), boost::bind( &ManipulationDataServer::icp_disconnection, this, _1));
    _move_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/move_pose", 1);
    _move_pose_dual_pub = _node.advertise<geometry_msgs::PoseArray>("/move_dual_pose", 1);
    _box_pub = _node.advertise<jsk_recognition_msgs::BoundingBox>("/particle_filter_tracker/renew_box", 1);
    _feedback_pub = _node.advertise<visualization_msgs::InteractiveMarkerFeedback>("/interactive_point_cloud/feedback", 1);
    usleep(100000);

    _debug_grasp_pub = _node.advertise<visualization_msgs::Marker>("/debug_grasp", 1);
    _marker_set_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/interactive_point_cloud/set_marker_pose", 1);
    _t_marker_set_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/transformable_interactive_server/set_pose", 1);
    _t_marker_box_pub = _node.advertise<jsk_recognition_msgs::BoundingBox>("/passed_selected_box", 1);
    _t_marker_information_pub = _node.advertise<drc_task_common::TMarkerInfo>("/t_marker_information", 1);
		subscribe_cloud_and_box();
    _icp_client = _node.serviceClient<jsk_recognition_msgs::ICPAlignWithBox>("/icp_registration/icp_service");
    _get_type_client = _node.serviceClient<jsk_interactive_marker::GetType>("/transformable_interactive_server/get_type");
    _get_pose_client = _node.serviceClient<jsk_interactive_marker::GetTransformableMarkerPose>("/transformable_interactive_server/get_pose");
    _get_dim_client = _node.serviceClient<jsk_interactive_marker::GetMarkerDimensions>("/transformable_interactive_server/get_dimensions");
    _tf_publish_client = _node.serviceClient<dynamic_tf_publisher::SetDynamicTF>("/manipulation_data_server/set_dynamic_tf");
    _subSelectedPoints = _node.subscribe("/selected_points",1,&ManipulationDataServer::set_menu_cloud,this);
    _subSelectedPose = _node.subscribe("/interactive_point_cloud/left_click_point_relative", 1, &ManipulationDataServer::set_menu_point, this);
    _sub_pose_feedback = _node.subscribe("/interactive_point_cloud/feedback", 1, &ManipulationDataServer::marker_move_feedback, this);
    _sub_object_pose_update = _node.subscribe("/simple_marker/update", 1, &ManipulationDataServer::t_marker_move_update, this);
    _sub_object_pose_feedback = _node.subscribe("/simple_marker/feedback", 1, &ManipulationDataServer::t_marker_move_feedback, this);
    _sub_grasp_pose_feedback = _node.subscribe("/grasp_pose_feedback", 1, &ManipulationDataServer::grasp_pose_feedback_cb, this);
    _sub_grasp_pose_feedback_not_allow_slip = _node.subscribe("/grasp_not_allow_slip_pose_feedback", 1, &ManipulationDataServer::grasp_not_allow_slip_pose_feedback_cb, this);
    _sub_grasp_pose_dual_feedback = _node.subscribe("/grasp_pose_dual_feedback", 1, &ManipulationDataServer::grasp_pose_dual_feedback_cb, this);
    _align_icp_server = _node.advertiseService("icp_apply", &ManipulationDataServer::align_cb, this);
    _save_server = _node.advertiseService("save_manipulation", &ManipulationDataServer::save_cb, this);
    _assoc_server = _node.advertiseService("assoc_points", &ManipulationDataServer::assoc_object_to_marker_cb, this);
    _disassoc_server = _node.advertiseService("disassoc_points", &ManipulationDataServer::disassoc_object_to_marker_cb, this);
    pub_tf();

  }
  void subscribe_cloud_and_box(){
    if(0){
      _subPoints.subscribe(_node , "/selected_pointcloud", 1);
      _subBox.subscribe(_node, "/bounding_box_marker/selected_box", 1);
      _sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      _sync->connectInput(_subPoints, _subBox);
      _sync->registerCallback(boost::bind(
					  &ManipulationDataServer::set_reference,
					  this, _1, _2));
    }else{ // for drc_comm
      _subICP.subscribe(_node, "/icp_registration/icp_result", 1);
      _subBox.subscribe(_node, "/bounding_box_marker/selected_box", 1);
      _sync_b_i = boost::make_shared<message_filters::Synchronizer<SyncPolicy_b_i> >(100);
      _sync_b_i->connectInput(_subBox, _subICP);
      _sync_b_i->registerCallback(boost::bind(
					  &ManipulationDataServer::set_with_icp_result,
					  this, _1, _2));
      _sub_clouds = _node.subscribe("/selected_pointcloud", 1, &ManipulationDataServer::set_cloud_cb, this);
    }
  }
  void init_reference(){
    std::vector<std::string> pcd_files;
    if (!jsk_topic_tools::readVectorParameter(_node, "models", pcd_files)){
      ROS_INFO("can not read param");
      pcd_files.resize(0);
      if(pcd_files.size() == 0) {
	ROS_INFO("no models is specified");
      }
    }
    for(size_t i=0; i<pcd_files.size(); i++){
      boost::shared_ptr<ManipulationData> temp_marker(new ManipulationData);
      if(read_marker(pcd_files[i].c_str(), *temp_marker)){
	_manip_data_array.push_back(temp_marker);
      }
    }
  }
  void pub_reference(){
    jsk_recognition_msgs::PointsArray reset_points;
    pcl::PointCloud <pcl::PointXYZRGB> all_points;
    for(size_t i=0; i<_manip_data_array.size(); i++){
      boost::shared_ptr<ManipulationData> temp_marker = _manip_data_array[i];
      sensor_msgs::PointCloud2 temp_cloud_msg;
      all_points += temp_marker->cloud;
      pcl::toROSMsg(temp_marker->cloud, temp_cloud_msg);
      reset_points.cloud_list.push_back(temp_cloud_msg);
    }
    //_pointsArrayPub.publish(temp_cloud_msg);
    sensor_msgs::PointCloud2 temp_cloud_msg_all;
    pcl::toROSMsg(all_points, temp_cloud_msg_all);
    temp_cloud_msg_all.header.frame_id = "manipulate_frame";
    temp_cloud_msg_all.header.stamp = ros::Time::now();
    _debug_cloud_pub.publish(temp_cloud_msg_all);
    _reset_pub.publish(reset_points);
    ROS_INFO("initialize done");
  }
  void pub_tf(){
    dynamic_tf_publisher::SetDynamicTF set_tf_srv;
    geometry_msgs::TransformStamped transform_msg_temp;
    set_tf_srv.request.freq = 20.0;
    transform_msg_temp.transform.translation.x = _tf_from_base.getOrigin().getX();
    transform_msg_temp.transform.translation.y = _tf_from_base.getOrigin().getY();
    transform_msg_temp.transform.translation.z = _tf_from_base.getOrigin().getZ();
    tf::Quaternion temp_qua;
    _tf_from_base.getBasis().getRotation(temp_qua);
    transform_msg_temp.transform.rotation.x = temp_qua.getX();
    transform_msg_temp.transform.rotation.y = temp_qua.getY();
    transform_msg_temp.transform.rotation.z = temp_qua.getZ();
    transform_msg_temp.transform.rotation.w = temp_qua.getW();
    transform_msg_temp.header.frame_id = std::string(_base_link_name);
    transform_msg_temp.child_frame_id = std::string("manipulate_frame");
    set_tf_srv.request.cur_tf = transform_msg_temp;
    if(!_tf_publish_client.call(set_tf_srv)){
      return;
    }
    tf::Transform _tf_marker_from_base = _tf_from_base*_tf_marker;
    transform_msg_temp.transform.translation.x = _tf_marker_from_base.getOrigin().getX();
    transform_msg_temp.transform.translation.y = _tf_marker_from_base.getOrigin().getY();
    transform_msg_temp.transform.translation.z = _tf_marker_from_base.getOrigin().getZ();
    _tf_marker_from_base.getBasis().getRotation(temp_qua);
    transform_msg_temp.transform.rotation.x = temp_qua.getX();
    transform_msg_temp.transform.rotation.y = temp_qua.getY();
    transform_msg_temp.transform.rotation.z = temp_qua.getZ();
    transform_msg_temp.transform.rotation.w = temp_qua.getW();
    transform_msg_temp.header.frame_id = std::string(_base_link_name);
    transform_msg_temp.child_frame_id = std::string("marker_frame");
    set_tf_srv.request.cur_tf = transform_msg_temp;
    if(!_tf_publish_client.call(set_tf_srv)){
      return;
    }
  }
  bool save_cb(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
  {
    save_marker();
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*_reference_cloud, ros_output);
    ros_output.header.stamp = ros::Time::now();
    ros_output.header.frame_id = "manipulate_frame";
    _debug_cloud_pub.publish(ros_output);
    if(!_reference_hit){
      _pointsArrayPub.publish(ros_output);
      _reference_hit = true;
      _manip_data_array.push_back(_manip_data_ptr);
    }
    else{
      ROS_INFO("before cloud reused");
    }
    return true;
  }
  bool save_marker(){
    if(_manip_data_ptr){
      InteractiveMarker int_marker_tmp;
      _manip_data_ptr->int_marker_array.int_markers.resize(0);
      if(_server->get("first_menu", int_marker_tmp)){
	_manip_data_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
      }
      if(_server->get("grasp_pose", int_marker_tmp)){
	_manip_data_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
      }
      if(_server->get("grasp_pose_2", int_marker_tmp)){
	_manip_data_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
      }
      if(_server->get("push_pose", int_marker_tmp)){
	_manip_data_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
      }
      if(_server->get("axial_restraint", int_marker_tmp)){
	_manip_data_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
      }
      _manip_data_ptr->cloud = *_reference_cloud;
      jsk_interactive_marker::GetMarkerDimensions get_dim_srv;
      jsk_interactive_marker::GetTransformableMarkerPose get_pose_srv;
      jsk_interactive_marker::GetType get_type_srv;
      get_dim_srv.request.target_name = get_pose_srv.request.target_name = "";
      if (_get_dim_client.call(get_dim_srv)){_manip_data_ptr->dim=get_dim_srv.response.dimensions;}
      if (_get_type_client.call(get_type_srv)){_manip_data_ptr->dim.type=get_type_srv.response.type;}
      if (_get_pose_client.call(get_pose_srv)){
        ROS_INFO("pose service succeed");
	geometry_msgs::PoseStamped temp_pose_stamped;
	//listener.transformPose("manipulate_frame", get_pose_srv.response.pose_stamped, temp_pose_stamped);
        ros::Time now = ros::Time::now();
        try{
          _listener.waitForTransform("manipulate_frame", get_pose_srv.response.pose_stamped.header.frame_id, now, ros::Duration(2.0));
          _listener.transformPose("manipulate_frame", now, get_pose_srv.response.pose_stamped,get_pose_srv.response.pose_stamped.header.frame_id, temp_pose_stamped);
          _manip_data_ptr->pose=temp_pose_stamped.pose;    
        }
        catch (tf::TransformException ex){
          ROS_ERROR("orientations may be 0s %s",ex.what());
          _manip_data_ptr->pose.orientation.x = 0;
          _manip_data_ptr->pose.orientation.y = 0;
          _manip_data_ptr->pose.orientation.z = 0;
          _manip_data_ptr->pose.orientation.w = 1;
        }

      }  
      else{
        ROS_INFO("save failed");
        _manip_data_ptr->pose.orientation.x = 0;
        _manip_data_ptr->pose.orientation.y = 0;
        _manip_data_ptr->pose.orientation.z = 0;
        _manip_data_ptr->pose.orientation.w = 1;
      }
      // service call current pose and box
      // change pose and box to manipulate_frame
      write_marker(_reference_num, *_manip_data_ptr);
      ROS_INFO("save succeeded %d", _reference_num);
      return true;
    }else{
      return false;
    }
  }
  void reverse_hand_menu_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    reverse_hand(feedback->marker_name);
  }

  void reverse_hand(std::string marker_name){
    //get grab marker
    InteractiveMarker int_marker_tmp;
    _server->get(marker_name, int_marker_tmp);
    int_marker_tmp.pose = tf_to_pose(pose_to_tf(int_marker_tmp.pose)*tf::Transform(tf::Quaternion(1, 0, 0, 0)));
    _server->insert(int_marker_tmp);
    _server->applyChanges();
  }
  void grasp_not_allow_slip_pose_feedback_cb(geometry_msgs::PoseStamped pose_msg){
    //change grasp pose
    InteractiveMarker int_marker_tmp;
    if(_server->get("grasp_pose", int_marker_tmp)){
      geometry_msgs::PoseStamped handle_pose_temp;
      ros::Time now = ros::Time::now();
      _listener.waitForTransform("/manipulate_frame", pose_msg.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame",now, pose_msg , pose_msg.header.frame_id, handle_pose_temp);
      tf::Transform handle_transform = pose_to_tf(handle_pose_temp.pose) * pose_to_tf(int_marker_tmp.pose).inverse();
      _tf_from_base = _tf_from_base * handle_transform;
      pub_tf();
      _timer_count=0;
    }
  }
  void grasp_pose_feedback_cb(geometry_msgs::PoseStamped pose_msg){
    //change grasp pose
    InteractiveMarker int_marker_tmp;
    if(_server->get("grasp_pose", int_marker_tmp)){
      geometry_msgs::PoseStamped handle_pose_temp;
      ros::Time now = ros::Time::now();
      _listener.waitForTransform("/manipulate_frame", pose_msg.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame",now, pose_msg , pose_msg.header.frame_id, handle_pose_temp);
      int_marker_tmp.pose = handle_pose_temp.pose;
      _grasp_pose = handle_pose_temp.pose;
      _server->insert(int_marker_tmp);
      _server->applyChanges();
    }
  }
  void grasp_pose_dual_feedback_cb(geometry_msgs::PoseArray pose_array_msg){
    //change grasp pose
    InteractiveMarker int_marker_tmp;
    if(_server->get("grasp_pose", int_marker_tmp)){
      geometry_msgs::PoseStamped handle_pose_temp;
      ros::Time now = ros::Time::now();
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.pose = pose_array_msg.poses[0];
      pose_msg.header = pose_array_msg.header;
      _listener.waitForTransform("/manipulate_frame", pose_msg.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame",now, pose_msg , pose_msg.header.frame_id, handle_pose_temp);
      int_marker_tmp.pose = handle_pose_temp.pose;
      _grasp_pose = handle_pose_temp.pose;
      _server->insert(int_marker_tmp);
      _server->applyChanges();
    }
    if(_server->get("grasp_pose_2", int_marker_tmp)){
      geometry_msgs::PoseStamped handle_pose_temp;
      ros::Time now = ros::Time::now();
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.pose = pose_array_msg.poses[1];
      pose_msg.header = pose_array_msg.header;
      _listener.waitForTransform("/manipulate_frame", pose_msg.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame",now, pose_msg , pose_msg.header.frame_id, handle_pose_temp);
      int_marker_tmp.pose = handle_pose_temp.pose;
      _server->insert(int_marker_tmp);
      _server->applyChanges();
    }

  }
  void icp_connection(const ros::SingleSubscriberPublisher& pub){
    pub_reference();
    ROS_INFO("icp_server_connected");
  }
  void icp_disconnection(const ros::SingleSubscriberPublisher& pub){  
  }

  ~ManipulationDataServer() {
  }
  void t_marker_move_update(const visualization_msgs::InteractiveMarkerUpdate update)
  {
    if(!_assoc_marker_flug) return;
    if(update.type == 0) return;
    if(update.markers.size()==1) {
      tf::Transform tf_to_object = pose_to_tf(update.markers[0].pose);
      geometry_msgs::PoseStamped marker_pose_stamped;
      marker_pose_stamped.pose = tf_to_pose( tf_to_object * _tf_object_constraint.inverse());
      marker_pose_stamped.header = update.markers[0].header;
      _marker_set_pose_pub.publish(marker_pose_stamped);
      geometry_msgs::PoseStamped marker_pose_stamped_from_manip;
      //_listener.transformPose("manipulate_frame", marker_pose_stamped, marker_pose_stamped_from_manip);
      ros::Time now = ros::Time::now();
      _listener.waitForTransform("manipulate_frame", marker_pose_stamped.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame",now, marker_pose_stamped , marker_pose_stamped.header.frame_id, marker_pose_stamped_from_manip);
      _tf_marker=pose_to_tf(marker_pose_stamped_from_manip.pose);
      pub_tf();
    }
  }
  void t_marker_move_feedback(const visualization_msgs::InteractiveMarkerFeedback feedback)
  {
    if(!_assoc_marker_flug) return;
    if(feedback.event_type == 1) {
      tf::Transform tf_to_object = pose_to_tf(feedback.pose);
      geometry_msgs::PoseStamped marker_pose_stamped;
      marker_pose_stamped.pose = tf_to_pose( tf_to_object * _tf_object_constraint.inverse());
      marker_pose_stamped.header = feedback.header;
      _marker_set_pose_pub.publish(marker_pose_stamped);
      geometry_msgs::PoseStamped marker_pose_stamped_from_manip;
      //_listener.transformPose("/manipulate_frame", marker_pose_stamped, marker_pose_stamped_from_manip);
      ros::Time now = ros::Time::now();
      _listener.waitForTransform("manipulate_frame", marker_pose_stamped.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("/manipulate_frame", now, marker_pose_stamped , marker_pose_stamped.header.frame_id, marker_pose_stamped_from_manip);
      _tf_marker=pose_to_tf(marker_pose_stamped_from_manip.pose);
      pub_tf();
    }
  }
  void revise_model_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    geometry_msgs::PoseStamped grasp_marker_before_pose, grasp_marker_after_pose;
    grasp_marker_before_pose.header.frame_id = "manipulate_frame";
    grasp_marker_before_pose.pose = _grasp_pose;//feedback->pose;
    visualization_msgs::InteractiveMarker int_marker_tmp;
    if(_server->get("grasp_pose", int_marker_tmp)){
      try{
        ros::Time now = ros::Time::now(); 
        _listener.waitForTransform(feedback->header.frame_id, "manipulate_frame", now, ros::Duration(2.0));
        _listener.transformPose(feedback->header.frame_id, now, grasp_marker_before_pose ,"manipulate_frame",  grasp_marker_after_pose); 
      }
      catch(tf::TransformException ex){
        ROS_ERROR("revise model failed %s",ex.what());
        return;
      }
      int_marker_tmp.pose = grasp_marker_after_pose.pose;
      ROS_INFO("get grasp succeeded");
      marker_move_function(feedback->header);
      _grasp_pose = grasp_marker_after_pose.pose;
      _server->insert(int_marker_tmp);
      _server->applyChanges();
    }else{
      ROS_INFO("get grasp failed");
      return;
    }
    if(_server->get("grasp_pose_2", int_marker_tmp)){
      try{
        grasp_marker_before_pose.header.frame_id = "manipulate_frame";
        grasp_marker_before_pose.pose = int_marker_tmp.pose;//feedback->pose;
        ros::Time now = ros::Time::now(); 
        _listener.waitForTransform(feedback->header.frame_id, "manipulate_frame", now, ros::Duration(2.0));
        _listener.transformPose(feedback->header.frame_id, now, grasp_marker_before_pose ,"manipulate_frame",  grasp_marker_after_pose); 
      }
      catch(tf::TransformException ex){
        ROS_ERROR("revise model failed %s",ex.what());
        return;
      }
      int_marker_tmp.pose = grasp_marker_after_pose.pose;
      ROS_INFO("get grasp succeeded");
      marker_move_function(feedback->header);
      _grasp_pose = grasp_marker_after_pose.pose;
      _server->insert(int_marker_tmp);
      _server->applyChanges();
    }else{
      ROS_INFO("get grasp failed");
      return;
    }

  }

  void marker_move_feedback(const visualization_msgs::InteractiveMarkerFeedback feedback)
  {
    //
    geometry_msgs::PoseStamped temp_pose_stamped, temp_feedback_pose_stamped;
    temp_feedback_pose_stamped.pose = feedback.pose;
    temp_feedback_pose_stamped.header = feedback.header;
    try{
      //_listener.transformPose("manipulate_frame", temp_feedback_pose_stamped, temp_pose_stamped);
      ros::Time now = ros::Time::now();
      _listener.waitForTransform("manipulate_frame", feedback.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("manipulate_frame", now, temp_feedback_pose_stamped , feedback.header.frame_id, temp_pose_stamped);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("marker move failed %s",ex.what());
      return;
    }

    _marker_pose = temp_pose_stamped.pose;
    //ROS_INFO("feedback_header %s", feedback.header.frame_id.c_str());
    //_listener.transformPose("manipulate_frame", feedback.pose,_marker_pose);
    tf::poseMsgToTF(_marker_pose, _tf_marker);
    pub_tf();
    //depends jsk_interactive_marker
    if(_assoc_marker_flug){
      geometry_msgs::PoseStamped t_pose_stamped;
      t_pose_stamped.pose = tf_to_pose(_tf_marker*_tf_object_constraint);
      t_pose_stamped.header = temp_pose_stamped.header;
      _t_marker_set_pose_pub.publish(t_pose_stamped);
    }
    if(feedback.menu_entry_id==1){
      marker_move_function(feedback.header);
    }
  }
 
  void move_hand_menu_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    if(feedback->marker_name=="grasp_pose")marker_move_function(feedback->header);
    if(feedback->marker_name=="grasp_pose_2")marker_move_dual_function(feedback->header);
  }
  void marker_move_function(std_msgs::Header header){
    geometry_msgs::PoseStamped move_pose;
    move_pose.header = header;
    move_pose.header.frame_id=std::string("marker_frame");
    move_pose.pose = _grasp_pose;
    _move_pose_pub.publish(move_pose);
    //pub zero feedback
    pub_zero_feedback(header);
  }
  void marker_move_dual_function(std_msgs::Header header){
    visualization_msgs::InteractiveMarker int_marker_tmp;
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header = header;
    if (_server->get("grasp_pose", int_marker_tmp)){
      pose_array_msg.header.frame_id = int_marker_tmp.header.frame_id;
      pose_array_msg.poses.push_back(int_marker_tmp.pose);
    }else return;
    if (_server->get("grasp_pose_2", int_marker_tmp)){
      pose_array_msg.poses.push_back(int_marker_tmp.pose);
    }else return;
    _move_pose_dual_pub.publish(pose_array_msg);
    //pub zero feedback
    pub_zero_feedback(header);
  }
  void pub_zero_feedback(std_msgs::Header header){
    _marker_pose.position.x=_marker_pose.position.y=_marker_pose.position.z=0;
    _marker_pose.orientation.x=_marker_pose.orientation.y=_marker_pose.orientation.z=0;_marker_pose.orientation.w=1;    
    sensor_msgs::PointCloud2 manip_cloud_msg;
    pcl::toROSMsg(*_reference_cloud, manip_cloud_msg);
    manip_cloud_msg.header = header;
    _pointsPub.publish(manip_cloud_msg);
    _tf_from_base = _tf_from_base * _tf_marker;
    tf::poseMsgToTF(_marker_pose, _tf_marker);
    pub_tf();
    _timer_count = 0;
  }
  void set_pose_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    tf::Transform tfTransformation;
    tf::StampedTransform tfTransformationStamped;
    ros::Time now = ros::Time::now();
    try {
      _listener.waitForTransform(_base_link_name, "track_result", now, ros::Duration(2.0));
      _listener.lookupTransform(_base_link_name, "track_result", now, tfTransformationStamped); 
      tf::Transform tfTransformation_diff =  _tf_from_base.inverse()*tfTransformationStamped;
      visualization_msgs::InteractiveMarker int_marker_tmp;
      if(_server->get("grasp_pose", int_marker_tmp)){
        int_marker_tmp.pose = tf_to_pose(tfTransformation_diff.inverse()*pose_to_tf(int_marker_tmp.pose));
        ROS_INFO("get grasp succeeded");
        _grasp_pose = int_marker_tmp.pose;
        _server->insert(int_marker_tmp);
        _server->applyChanges();
      }else{
      }
      _tf_from_base = (tf::Transform) tfTransformationStamped;
      pub_tf();
    }
    catch(tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }

  void set_grasp_pose_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    tf::Transform tfTransformation;
    tf::StampedTransform tfTransformationStamped;
    ros::Time now = ros::Time::now();
    try {
      ros::ServiceClient client = _node.serviceClient<drc_task_common::GetIKArmPose>("get_ik_arm_pose");
      drc_task_common::GetIKArmPose srv;
      if(!client.call(srv)){
        ROS_INFO("cannot get ik arm pose");
        return;
      }
      geometry_msgs::PoseStamped grasp_pose_stamped;
      _listener.waitForTransform("manipulate_frame", srv.response.pose_stamped.header.frame_id, now, ros::Duration(2.0));
      _listener.transformPose("manipulate_frame",now, srv.response.pose_stamped, srv.response.pose_stamped.header.frame_id, grasp_pose_stamped);
      visualization_msgs::InteractiveMarker int_marker_tmp;
      if(_server->get("grasp_pose", int_marker_tmp)){
        int_marker_tmp.pose = grasp_pose_stamped.pose;
        ROS_INFO("get grasp succeeded");
        _grasp_pose = grasp_pose_stamped.pose;
        _server->insert(int_marker_tmp);
        _server->applyChanges();
      }else{
      }
    }
    catch(tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  }
  void reset_pose_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    std_msgs::String a;
    _reset_pose_pub.publish(a);
  }
  void push_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("push_cb driven");
    //remove circle marker
    _server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.14;
    int_marker.name = "push_pose";
    int_marker.pose = _menu_pose;
    visualization_msgs::InteractiveMarkerControl push_control;
    push_control.always_visible = true;
    push_control.markers.push_back(make_arrow(int_marker));
    
    push_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls.push_back(push_control);
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    _server->insert(int_marker);
    _menu_handler_push.apply( *_server , "push_pose");
    _server->applyChanges();
  }
  void axial_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    //remove circle marker
    _server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.14;
    int_marker.name = "axial_restraint";
    int_marker.pose.position = _menu_pose.position;
    if(_reference_box_size.x > _reference_box_size.y && _reference_box_size.x > _reference_box_size.z){
      if (_menu_pose.position.x < 0){
	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 1;
	int_marker.pose.orientation.z = 0;
	int_marker.pose.orientation.w = 0;
      }else{
       	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 0;
	int_marker.pose.orientation.z = 0;
	int_marker.pose.orientation.w = 1;
      }
    }else if(_reference_box_size.y > _reference_box_size.z){
      if (_menu_pose.position.y < 0){
	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 0;
	int_marker.pose.orientation.z = -0.7071;
	int_marker.pose.orientation.w = 0.7071;
      }else{
	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 0;
	int_marker.pose.orientation.z = 0.7071;
	int_marker.pose.orientation.w = 0.7071;
      }
    }
    else{
      if (_menu_pose.position.z < 0){
	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = 0.7071;
	int_marker.pose.orientation.z = 0;
	int_marker.pose.orientation.w = 0.7071;
      }else{
	int_marker.pose.orientation.x = 0;
	int_marker.pose.orientation.y = -0.7071;
	int_marker.pose.orientation.z = 0;
	int_marker.pose.orientation.w = 0.7071;
      }
    }
    if(_all_manual){
      int_marker.pose.orientation.x = 0;
      int_marker.pose.orientation.y = 0;
      int_marker.pose.orientation.z = 0;
      int_marker.pose.orientation.w = 1;
    }
    visualization_msgs::InteractiveMarkerControl axial_control;
    axial_control.always_visible = true;
    axial_control.markers.push_back(make_arrow(int_marker, 0, 1.0, 0.5, 0.5));
    axial_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls.push_back(axial_control);
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    _server->insert(int_marker);
    _menu_handler_axial_restraint.apply( *_server , "axial_restraint");
    _server->applyChanges();
  }
  void remove_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("remove_cb driven");
    _server->erase(feedback->marker_name);
    _server->applyChanges();
  }
  void do_grasp_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    if (feedback->marker_name=="grasp_pose"){
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = feedback->header;
      pose_msg.header.frame_id = std::string("manipulate_frame");
      pose_msg.pose = feedback->pose;
      _grasp_pose_pub.publish(pose_msg);
      // mode change 
      _grasp_pose = feedback->pose;
      // insert axial restraint
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = feedback->header.frame_id;
      int_marker.scale = 0.14;
      int_marker.name = "axial_restraint";
      int_marker.pose = tf_to_pose(pose_to_tf(_grasp_pose) * tf::Transform(tf::Quaternion(1, 0, 0, 0)) *tf::Transform(tf::Quaternion(0, 0.7071, 0, 0.7071)));//_grasp_pose.position;
      if(_all_manual){
        int_marker.pose.orientation.x = 0;
      int_marker.pose.orientation.y = 0;
      int_marker.pose.orientation.z = 0;
      int_marker.pose.orientation.w = 1;
      }
      visualization_msgs::InteractiveMarkerControl axial_control;
      axial_control.always_visible = true;
      axial_control.markers.push_back(make_arrow(int_marker, 0, 1.0, 0.5, 0.5, 0.5, 0.3));
      axial_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
      int_marker.controls.push_back(axial_control);
      visualization_msgs::InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      _server->insert(int_marker);
      _menu_handler_axial_restraint.apply( *_server , "axial_restraint");
    _server->applyChanges();
    }
    else {
      // _grasp_pose_2 will be reserved, but for now, it is not needed
      InteractiveMarker int_marker_tmp;
      if (_server->get("grasp_pose", int_marker_tmp)){
        _grasp_pose = int_marker_tmp.pose;
        geometry_msgs::PoseArray pose_array_msg;
        pose_array_msg.header = feedback->header;
        pose_array_msg.header.frame_id = std::string("manipulate_frame");
        pose_array_msg.poses.push_back(int_marker_tmp.pose);
        pose_array_msg.poses.push_back(feedback->pose);
        _grasp_pose_dual_pub.publish(pose_array_msg);
      } 
      else return;
    }
  }
  void do_grasp_z_free_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    if(feedback->marker_name=="grasp_pose"){
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = feedback->header;
      pose_msg.header.frame_id = std::string("manipulate_frame");
      pose_msg.pose = feedback->pose;
      _grasp_pose_z_free_pub.publish(pose_msg);
      // mode change 
      _grasp_pose = feedback->pose;
      // insert axial restraint
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = feedback->header.frame_id;
      int_marker.scale = 0.14;
      int_marker.name = "axial_restraint";
      int_marker.pose = tf_to_pose(pose_to_tf(_grasp_pose) * tf::Transform(tf::Quaternion(1, 0, 0, 0)) *tf::Transform(tf::Quaternion(0, 0.7071, 0, 0.7071)));//_grasp_pose.position;
      if(_all_manual){
        int_marker.pose.orientation.x = 0;
        int_marker.pose.orientation.y = 0;
        int_marker.pose.orientation.z = 0;
        int_marker.pose.orientation.w = 1;
      }
      visualization_msgs::InteractiveMarkerControl axial_control;
      axial_control.always_visible = true;
      axial_control.markers.push_back(make_arrow(int_marker, 0, 1.0, 0.5, 0.5, 0.5, 0.3));
      axial_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
      int_marker.controls.push_back(axial_control);
      visualization_msgs::InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      _server->insert(int_marker);
      _menu_handler_axial_restraint.apply( *_server , "axial_restraint");
      _server->applyChanges();
    }
    if(feedback->marker_name=="grasp_pose_2"){
      InteractiveMarker int_marker_tmp;
      if (_server->get("grasp_pose", int_marker_tmp)){
        _grasp_pose = int_marker_tmp.pose;
        geometry_msgs::PoseArray pose_array_msg;
        pose_array_msg.header = feedback->header;
        pose_array_msg.header.frame_id = std::string("manipulate_frame");
        pose_array_msg.poses.push_back(int_marker_tmp.pose);
        pose_array_msg.poses.push_back(feedback->pose);
        _grasp_pose_dual_z_free_pub.publish(pose_array_msg);
      } 
    }
  }
  void do_push_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = feedback->header;
    pose_msg.header.frame_id = std::string("manipulate_frame");
    pose_msg.pose = feedback->pose;
    _push_pose_pub.publish(pose_msg);
  }
  void do_push_with_assist_many_times_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::PoseArray pose_array_msg;
    pose_msg.header = feedback->header;
    pose_msg.header.frame_id = std::string("manipulate_frame");
    pose_msg.pose = feedback->pose;
    pose_array_msg.header = pose_msg.header;
    pose_array_msg.poses.push_back(pose_msg.pose);
    pose_array_msg.poses.push_back(_grasp_pose);
    _push_pose_with_assist_many_times_pub.publish(pose_array_msg);
  }
  void do_push_with_assist_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::PoseArray pose_array_msg;
    pose_msg.header = feedback->header;
    pose_msg.header.frame_id = std::string("manipulate_frame");
    pose_msg.pose = feedback->pose;
    pose_array_msg.header = pose_msg.header;
    pose_array_msg.poses.push_back(pose_msg.pose);
    pose_array_msg.poses.push_back(_grasp_pose);
    _push_pose_with_assist_pub.publish(pose_array_msg);
  }
  geometry_msgs::PoseArray do_move_by_axial_restraion_common(std_msgs::Header header, geometry_msgs::Pose pose){
    geometry_msgs::PoseArray move_pose_array;
    move_pose_array.header = header;
    move_pose_array.header.frame_id=std::string("marker_frame");
    move_pose_array.poses.push_back(_grasp_pose);
    move_pose_array.poses.push_back(pose);
    //_move_pose_pub.publish(move_pose);
    
    //pub zero feedback
    _marker_pose.position.x=_marker_pose.position.y=_marker_pose.position.z=0;
    _marker_pose.orientation.x=_marker_pose.orientation.y=_marker_pose.orientation.z=0;_marker_pose.orientation.w=1;
    sensor_msgs::PointCloud2 manip_cloud_msg;
    pcl::toROSMsg(*_reference_cloud, manip_cloud_msg);
    manip_cloud_msg.header = header;
    _pointsPub.publish(manip_cloud_msg);
    _tf_from_base = _tf_from_base * _tf_marker;
    tf::poseMsgToTF(_marker_pose, _tf_marker);
    pub_tf();
    _timer_count = 0;
    return move_pose_array;
  }
  void do_move_by_axial_restraion_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    _move_by_axial_restraint_pose_pub.publish(do_move_by_axial_restraion_common(feedback->header, feedback->pose));
  }
  void do_move_by_axial_restraion_not_allow_slip_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    _move_by_axial_restraint_not_allow_slip_pose_pub.publish(do_move_by_axial_restraion_common(feedback->header, feedback->pose));
  }
  void grasp_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    ROS_INFO("grasp_cb driven");
    //remove circle marker
    _server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.2;
    int_marker.name = "grasp_pose";
    int_marker.pose = _menu_pose;
    visualization_msgs::InteractiveMarkerControl grab_control;
    grab_control.always_visible = true;
    std::vector <visualization_msgs::Marker> grab_marker = make_grab(int_marker); 
    grab_control.markers.insert(grab_control.markers.end(), grab_marker.begin(), grab_marker.end());
    grab_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls.push_back(grab_control);
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  
    _server->insert(int_marker);
    _menu_handler_grasp.apply( *_server , "grasp_pose");
    _server->applyChanges();
  }
  void grasp_cb_2(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("grasp_cb_2 driven");
    //remove circle marker
    _server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.2;
    int_marker.name = "grasp_pose_2";
    int_marker.pose = _menu_pose;
    visualization_msgs::InteractiveMarkerControl grab_control;
    grab_control.always_visible = true;
    std::vector <visualization_msgs::Marker> grab_marker = make_grab(int_marker, 0.5);
    grab_control.markers.insert(grab_control.markers.end(), grab_marker.begin(), grab_marker.end());
    grab_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls.push_back(grab_control);
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    _server->insert(int_marker);
    _menu_handler_grasp.apply( *_server , "grasp_pose_2");
    _server->applyChanges();
  }
  
  bool assoc_object_to_marker_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    return assoc_object_to_marker();
  }
  bool assoc_object_to_marker(){
    _assoc_marker_flug=true;
    if(_assoc_marker_flug){
      jsk_interactive_marker::GetTransformableMarkerPose get_pose_srv;
      get_pose_srv.request.target_name="";
      if(_get_pose_client.call(get_pose_srv)){
	geometry_msgs::PoseStamped after_pose_;
	//_listener.transformPose("marker_frame", get_pose_srv.response.pose_stamped, after_pose_);
        ros::Time now = ros::Time::now();
        _listener.waitForTransform("marker_frame", get_pose_srv.response.pose_stamped.header.frame_id, now, ros::Duration(2.0));
	_listener.transformPose("marker_frame",now, get_pose_srv.response.pose_stamped, get_pose_srv.response.pose_stamped.header.frame_id, after_pose_);
	_tf_object_constraint = pose_to_tf(after_pose_.pose);
      }
    }
    return true;
  }
  
  bool disassoc_object_to_marker_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    return disassoc_object_to_marker();
  }
  bool disassoc_object_to_marker(){
    _assoc_marker_flug=false;
    return true;
  }
  jsk_recognition_msgs::ICPResult request_icp(sensor_msgs::PointCloud2 *cloud_msg_ptr, jsk_recognition_msgs::BoundingBox *box_msg_ptr){
    jsk_recognition_msgs::ICPAlignWithBox srv; 
    srv.request.target_cloud = *cloud_msg_ptr;
    srv.request.target_box = *box_msg_ptr;
    if (_icp_client.call(srv)){
      return srv.response.result;
    }
    else{
      jsk_recognition_msgs::ICPResult icp_result;
      icp_result.score = 1.0;
      icp_result.header = box_msg_ptr->header;
      icp_result.pose = box_msg_ptr->pose;
      icp_result.name = std::string("NONE");
      return icp_result;
    }
  }
  void pub_t_marker(jsk_recognition_msgs::BoundingBox& box, jsk_recognition_msgs::ICPResult& icp_result){
    if(_reference_hit){   
      geometry_msgs::PoseStamped temp_pose_stamped;
      geometry_msgs::Pose temp_pose, marker_pose = _manip_data_ptr->pose;
      tf::Transform tf_transformable_marker(tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w), tf::Vector3(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z));
      tf::Transform tf_marker_to_camera = _tf_from_camera * tf_transformable_marker;
      //pub marker
      drc_task_common::TMarkerInfo t_marker_info;
      t_marker_info.marker_pose_stamped.pose = tf_to_pose(tf_marker_to_camera);
      t_marker_info.marker_pose_stamped.header = box.header;
      t_marker_info.marker_dim = _manip_data_ptr->dim;
      _t_marker_information_pub.publish(t_marker_info);
    }
    else{
      //pub b_box
      _t_marker_box_pub.publish(box);
    }
  }

  bool align_cb(drc_task_common::ICPService::Request& req,
		drc_task_common::ICPService::Response& res){
    jsk_recognition_msgs::ICPResult icp_result = request_icp(&(req.points), &(req.box));
    set_icp((req.box), icp_result);
    set_point_cloud(&(req.points), _tf_from_base);
    if(_reference_hit){
      geometry_msgs::PoseStamped temp_pose_stamped;
      geometry_msgs::Pose temp_pose, marker_pose = _manip_data_ptr->pose;
      tf::Transform tf_transformable_marker(tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w), tf::Vector3(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z));
      tf::Transform tf_marker_to_camera = _tf_from_camera * tf_transformable_marker;
      temp_pose_stamped.header = req.points.header;
      temp_pose_stamped.pose = tf_to_pose(tf_marker_to_camera);
      res.dim = _manip_data_ptr->dim;
      res.pose_stamped = temp_pose_stamped;
      return true;
    }else{
      return false;
    }
  }

  void set_reference(const sensor_msgs::PointCloud2ConstPtr& msg_ptr, const jsk_recognition_msgs::BoundingBoxConstPtr& box_ptr){
    sensor_msgs::PointCloud2 msg = *msg_ptr;
    jsk_recognition_msgs::BoundingBox box = *box_ptr;
    jsk_recognition_msgs::ICPResult icp_result = request_icp(&msg, &box);
    set_icp(box, request_icp(&msg, &box));
    set_point_cloud(&msg, _tf_from_base);
    pub_t_marker(box, icp_result);
  }
  void set_with_icp_result(const jsk_recognition_msgs::BoundingBoxConstPtr& box_ptr, const jsk_recognition_msgs::ICPResultConstPtr& icp_ptr){
    jsk_recognition_msgs::BoundingBox box = *box_ptr;
    jsk_recognition_msgs::ICPResult icp_result = *icp_ptr;
    set_icp(box, icp_result);
    //set_point_cloud(&msg, _tf_from_base);
    pub_t_marker(box, icp_result);
  }
  void set_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){
    sensor_msgs::PointCloud2 cloud = *cloud_ptr;
    set_point_cloud(&cloud, _tf_from_base);
  }

  void set_icp(jsk_recognition_msgs::BoundingBox& box, jsk_recognition_msgs::ICPResult icp_result){
    //sensor_msgs::PointCloud2* msg_ptr = &msg;
    jsk_recognition_msgs::BoundingBox* box_ptr = &box;
    //believes that header is same
    //maybe check diference, if there are none, box_ptr will be selected 
    //marker_init
    geometry_msgs::PoseStamped before_pose_, after_pose_;
    disassoc_object_to_marker();
    //erase_marker();
    _server->erase("first_menu");
    _server->erase("grasp_pose");
    _server->erase("grasp_pose_2");
    _server->erase("push_pose");
    _server->erase("axial_restraint");
    _server->applyChanges();
    ROS_INFO("after_request");
    if (icp_result.score < 0.000075 && icp_result.name != "NONE"){
      before_pose_.pose = icp_result.pose;
      before_pose_.header = icp_result.header;
      ROS_INFO("before pose driven");
      ROS_INFO("score was %f", icp_result.score);
      int num  = atoi(icp_result.name.c_str());
      ROS_INFO("_reference_num was %d", num);
      _reference_num = num;
      boost::shared_ptr<ManipulationData> markers = _manip_data_array[num];
      _manip_data_ptr = _manip_data_array[num];
      for(std::vector<InteractiveMarker>::iterator marker_one_ptr = markers->int_marker_array.int_markers.begin() ; marker_one_ptr!= markers->int_marker_array.int_markers.end() ;marker_one_ptr++){
	_server->insert(*marker_one_ptr);
      }
      _menu_handler_first.apply( *_server , "first_menu");
      _menu_handler_grasp.apply( *_server , "grasp_pose");
      _menu_handler_grasp.apply( *_server , "grasp_pose_2");
      _menu_handler_push.apply( *_server , "push_pose");
      _menu_handler_axial_restraint.apply( *_server , "axial_restraint");
      _server->applyChanges();
      _reference_hit=true;
      box.pose = icp_result.pose;
    }
    else {
      before_pose_.pose = box_ptr->pose;
      before_pose_.header = box_ptr->header;
      ROS_INFO("new pose");
      _manip_data_ptr = boost::make_shared<ManipulationData> ();
      //_manip_data_array.push_back(_manip_data_ptr);
      _reference_num = _manip_data_array.size();
      _reference_hit=false;
    }
    _box_pub.publish(box);
    //_listener.transformPose(_base_link_name, before_pose_, after_pose_);
    ros::Time now = ros::Time::now();
    _listener.waitForTransform(_base_link_name, before_pose_.header.frame_id, now, ros::Duration(2.0));
    _listener.transformPose(_base_link_name,ros::Time::now()-(ros::Duration(0.2)), before_pose_, before_pose_.header.frame_id, after_pose_);
    // _listener.waitForTransform(_base_link_name, before_pose_.header.frame_id, ros::Time(0), ros::Duration(10.0));
    // _listener.transformPose(_base_link_name, before_pose_,after_pose_);
    _tf_from_base.setOrigin(tf::Vector3(after_pose_.pose.position.x,after_pose_.pose.position.y, after_pose_.pose.position.z));
    _tf_from_base.setRotation(tf::Quaternion(after_pose_.pose.orientation.x, after_pose_.pose.orientation.y, after_pose_.pose.orientation.z, after_pose_.pose.orientation.w));
    _tf_from_camera.setOrigin(tf::Vector3(before_pose_.pose.position.x,before_pose_.pose.position.y, before_pose_.pose.position.z));
    _tf_from_camera.setRotation(tf::Quaternion(before_pose_.pose.orientation.x, before_pose_.pose.orientation.y, before_pose_.pose.orientation.z, before_pose_.pose.orientation.w));
    _tf_from_base = _tf_from_base;//*transform_camera_to_optical;
    _tf_marker = tf::Transform(tf::Quaternion(0, 0, 0, 1));
    pub_tf();
    _timer_count = 0; 
    //move point_cloud
    _reference_box_size = box_ptr->dimensions;
  }
  void set_point_cloud(sensor_msgs::PointCloud2 *msg_ptr, tf::Transform transform){
    try
    {
      Eigen::Affine3f offset;
      transform_pointcloud_in_frame(
        transform, *msg_ptr,
        *_reference_cloud, offset,
        _listener);
      Eigen::Affine3f offset_inverse = offset.inverse();
      
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			_reference_kd_tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB>);
			ne.setInputCloud(_reference_cloud);
			ne.setSearchMethod(_reference_kd_tree);
			ne.setRadiusSearch(0.02);
			ne.setViewPoint(offset.translation().x(), offset.translation().y(), offset.translation().z());//after_orig.point.x, after_orig.point.y, after_orig.point.z);
			//hoge
			_reference_cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
			ne.compute(*_reference_cloud_normals);
			pcl::PointCloud<pcl::PointXYZRGBNormal> concatenated_cloud;
			concatenated_cloud.points.resize(_reference_cloud->points.size());
			concatenated_cloud.width = _reference_cloud->width;
			concatenated_cloud.height = _reference_cloud->height;
			concatenated_cloud.is_dense = _reference_cloud->is_dense;
	
			for (size_t i = 0; i < concatenated_cloud.points.size(); i++) {
				pcl::PointXYZRGBNormal point;
				point.x = _reference_cloud->points[i].x;
				point.y = _reference_cloud->points[i].y;
				point.z = _reference_cloud->points[i].z;
				point.rgb = _reference_cloud->points[i].rgb;
				point.normal_x = _reference_cloud_normals->points[i].normal_x;
				point.normal_y = _reference_cloud_normals->points[i].normal_y;
				point.normal_z = _reference_cloud_normals->points[i].normal_z;
				point.curvature = _reference_cloud_normals->points[i].curvature;
				concatenated_cloud.points[i] = point;
			}
			sensor_msgs::PointCloud2 output_debug_cloud;
			pcl::toROSMsg(concatenated_cloud,output_debug_cloud);
			output_debug_cloud.header = msg_ptr->header;
			output_debug_cloud.header.frame_id = "manipulate_frame";
			_debug_cloud_pub.publish(output_debug_cloud);
			//pub_offset_pose_.publish(box_pose_respected_to_cloud);
		}
    catch (tf2::ConnectivityException &e)
		{
			ROS_INFO("Transform error: %s", e.what());
      }
    catch (tf2::InvalidArgumentException &e)
		{
			ROS_INFO("Transform error: %s", e.what());
		}  
		_timer_count = 0;
  }
  void set_menu(float p_x, float p_y, float p_z, std_msgs::Header header){
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = p_x;
    searchPoint.y = p_y;
    searchPoint.z = p_z;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (_reference_kd_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    }
    else{
      ROS_INFO("kdtree failed");
      return;
    }
    
    float x = _menu_pose.position.x=_reference_cloud->points[pointIdxNKNSearch[0]].x;
    float y = _menu_pose.position.y=_reference_cloud->points[pointIdxNKNSearch[0]].y; 
    float z = _menu_pose.position.z=_reference_cloud->points[pointIdxNKNSearch[0]].z; 
    float v_x= _reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_x;
    float v_y= _reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_y;
    float v_z= _reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_z;
    ROS_INFO("point, %f %f %f ,normal %f, x:%f, y:%f, z:%f" , x, y, z, NORM(v_x, v_y, v_z), v_x, v_y, v_z);
    

    double theta = acos(v_x);
    // later multipied
    tf::Quaternion normal_(0, v_z/NORM(0, v_y, v_z) * cos(theta/2), -v_y/NORM(0, v_y, v_z) * cos(theta/2), sin(theta/2));
    //double theta_r = 3.14/4;
    //tf::Quaternion rotate_(cos(theta_r), 0, 0, sin(theta_r));
    tf::Quaternion final_quaternion = normal_ ;//* rotate_;
    double finger_t = 0.02;
    double finger_w = 0.01;
    double arm_t = 0.05;
    double finger_d = 0.05;
    pcl::PointCloud<pcl::PointXYZRGB> points_all;
    double min_theta_index = 0;
    double min_width = 100;
    tf::Quaternion min_qua(0, 0, 0, 1);
    visualization_msgs::Marker hand_marker;
    hand_marker.header = header;
    hand_marker.header.frame_id = std::string("marker_frame");
    hand_marker.ns = string("debug_grasp");
    hand_marker.id = 0;
    hand_marker.type = visualization_msgs::Marker::LINE_LIST;
    hand_marker.pose.orientation.w = 1;
    hand_marker.scale.x=0.003;
    tf::Matrix3x3 best_mat;
    for(double theta_=0; theta_<3.14/2;
	  theta_+=3.14/2/30){
      tf::Quaternion rotate_(sin(theta_), 0, 0, cos(theta_));     
      tf::Quaternion temp_qua = normal_ * rotate_;
      // 平面にあたらないって条件で(両方
      tf::Matrix3x3 temp_mat(temp_qua);
      // ROS_INFO("normal_test %f, %f, %f",temp_mat.getColumn(0)[0],temp_mat.getColumn(0)[1],temp_mat.getColumn(0)[2]);
      // 変換して jsk_pcl_rosのあれ

      geometry_msgs::Pose pose_respected_to_tf;
      pose_respected_to_tf.position.x = x;
      pose_respected_to_tf.position.y = y;
      pose_respected_to_tf.position.z = z;
      pose_respected_to_tf.orientation.x = temp_qua.getX();
      pose_respected_to_tf.orientation.y = temp_qua.getY();
      pose_respected_to_tf.orientation.z = temp_qua.getZ();
      pose_respected_to_tf.orientation.w = temp_qua.getW();
      Eigen::Affine3d box_pose_respected_to_cloud_eigend;
      tf::poseMsgToEigen(pose_respected_to_tf,
			 box_pose_respected_to_cloud_eigend);
      Eigen::Affine3d box_pose_respected_to_cloud_eigend_inversed
	= box_pose_respected_to_cloud_eigend.inverse();
      Eigen::Matrix4f box_pose_respected_to_cloud_eigen_inversed_matrixf;
      Eigen::Matrix4d box_pose_respected_to_cloud_eigen_inversed_matrixd
	= box_pose_respected_to_cloud_eigend_inversed.matrix();
      jsk_pcl_ros::convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(
						       box_pose_respected_to_cloud_eigen_inversed_matrixd,
						       box_pose_respected_to_cloud_eigen_inversed_matrixf);
      Eigen::Affine3f offset = Eigen::Affine3f(box_pose_respected_to_cloud_eigen_inversed_matrixf);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*_reference_cloud, *output_cloud, offset);

      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_z(new pcl::PointCloud<pcl::PointXYZRGB>), points_yz(new pcl::PointCloud<pcl::PointXYZRGB>), points_xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
      pass.setInputCloud (output_cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-arm_t*3, arm_t*3);
      pass.filter(*points_z);
      pass.setInputCloud (points_z);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-finger_w, finger_w);
      pass.filter(*points_yz);
      pass.setInputCloud (points_yz);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-0.2, finger_d);
      pass.filter(*points_xyz);
      // kdtree作って
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      points_all+=*points_xyz;
      // y成分以外抜く
      for (size_t index=0; index<points_xyz->size(); index++){
	points_xyz->points[index].x = points_xyz->points[index].z = 0;
      }
      if(points_xyz->points.size() == 0){ROS_INFO("points are empty");return;}
      kdtree.setInputCloud(points_xyz);
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      pcl::PointXYZRGB search_point_tree;
      search_point_tree.x=search_point_tree.y=search_point_tree.z=0;
      // 距離見て
      // いい感じの距離。短くて隙間あり(if diff from before was > 3)
      if ( kdtree.radiusSearch (search_point_tree, 10, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
	double before_w=10, temp_w;
	for (size_t index = 0; index < pointIdxRadiusSearch.size (); ++index){
	  temp_w =sqrt(pointRadiusSquaredDistance[index]);
	  //ROS_INFO("temp_w %f", temp_w);
	  if(temp_w - before_w > finger_t){
	    ROS_INFO("break driven, index, %td", index);
	    break;
	  }
	  //ROS_INFO("temp_d: %f", temp_d);
	  before_w=temp_w;
	}
	if(before_w < min_width){
	  min_theta_index = theta_;
	  min_width = before_w;
	  min_qua = temp_qua;
	  best_mat = temp_mat;
	}
	geometry_msgs::Point temp_point;
	std_msgs::ColorRGBA temp_color;
	temp_color.r=0; temp_color.g=0; temp_color.b=1; temp_color.a=1;
	temp_point.x=x-temp_mat.getColumn(1)[0] * before_w;
	temp_point.y=y-temp_mat.getColumn(1)[1] * before_w;
	temp_point.z=z-temp_mat.getColumn(1)[2] * before_w;
	hand_marker.points.push_back(temp_point);
	hand_marker.colors.push_back(temp_color);

	temp_point.x+=2*temp_mat.getColumn(1)[0] * before_w;
	temp_point.y+=2*temp_mat.getColumn(1)[1] * before_w;
	temp_point.z+=2*temp_mat.getColumn(1)[2] * before_w;
	hand_marker.points.push_back(temp_point);
	hand_marker.colors.push_back(temp_color);
      }
      //ROS_INFO("min_rad=%f, min_width=%f", min_theta_index, min_width);
    }
    geometry_msgs::Point temp_point;
    std_msgs::ColorRGBA temp_color;
    temp_color.r=1; temp_color.g=0; temp_color.b=0; temp_color.a=1;
    temp_point.x=x; temp_point.y=y;temp_point.z=z;
    hand_marker.points.push_back(temp_point);
    hand_marker.colors.push_back(temp_color);  
    temp_point.x-=best_mat.getColumn(0)[0] * min_width;
    temp_point.y-=best_mat.getColumn(0)[1] * min_width;
    temp_point.z-=best_mat.getColumn(0)[2] * min_width;
    hand_marker.points.push_back(temp_point);
    hand_marker.colors.push_back(temp_color);
    ROS_INFO("min_width %f", min_width);
    temp_color.r=0; temp_color.g=1; temp_color.b=0; temp_color.a=1;
    temp_point.x=x-best_mat.getColumn(1)[0] * min_width;
    temp_point.y=y-best_mat.getColumn(1)[1] * min_width;
    temp_point.z=z-best_mat.getColumn(1)[2] * min_width;
    hand_marker.points.push_back(temp_point);
    hand_marker.colors.push_back(temp_color);
    temp_point.x+=2*best_mat.getColumn(1)[0] * min_width;
    temp_point.y+=2*best_mat.getColumn(1)[1] * min_width;
    temp_point.z+=2*best_mat.getColumn(1)[2] * min_width;
    hand_marker.points.push_back(temp_point);
    hand_marker.colors.push_back(temp_color);
    
    _debug_grasp_pub.publish(hand_marker);
    _menu_pose.orientation.x = min_qua.getX();
    _menu_pose.orientation.y = min_qua.getY();
    _menu_pose.orientation.z = min_qua.getZ();
    _menu_pose.orientation.w = min_qua.getW();
    if(_all_manual){
      _menu_pose.orientation.x = _menu_pose.orientation.y = _menu_pose.orientation.z = 0.0;
      _menu_pose.orientation.w = 1.0;
    }
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/marker_frame";
    int_marker.scale = 0.2;
    int_marker.name = "first_menu";

    int_marker.pose = _menu_pose;
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.markers.push_back( make_circle(int_marker ));
    int_marker.controls.push_back(control);
    
    _server->insert( int_marker );
    _menu_handler_first.apply( *_server , "first_menu");
    _server->applyChanges();    

  }
  void set_menu_point(const geometry_msgs::PointStampedPtr & msg_ptr)
  {
    ROS_INFO("points selected with marker");
    geometry_msgs::PointStamped after_point;
    //_listener.transformPoint("/manipulate_frame", ros::Time::now() - (ros::Duration(0.05)) ,*msg_ptr, msg_ptr->header.frame_id,  after_point);
    after_point = *msg_ptr;
    set_menu(after_point.point.x, after_point.point.y, after_point.point.z, msg_ptr->header);
    pcl::PointXYZRGB searchPoint;
  }
  
  void set_menu_cloud(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) // may be marker_
  { 
    ROS_INFO("points selected");
    // change frame
    if(_reference_cloud->points.size()==0){
      ROS_INFO("points is empty");
      return;
    }
    sensor_msgs::PointCloud before_cloud;
    sensor_msgs::PointCloud after_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg_ptr, before_cloud);
    _br.sendTransform(tf::StampedTransform(_tf_from_base, ros::Time::now(), _base_link_name, "marker_frame"));
    _listener.transformPointCloud("/marker_frame", before_cloud, after_cloud);
    sensor_msgs::PointCloud2 manip_cloud_msg;
    sensor_msgs::convertPointCloudToPointCloud2(after_cloud, manip_cloud_msg);
    // decide main point
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(manip_cloud_msg, pcl_cloud);
    pcl::PointXYZRGB point_main = pcl_cloud.points[0];
    set_menu(point_main.x, point_main.y, point_main.z, msg_ptr->header);
  }
  visualization_msgs::Marker makeCircle(visualization_msgs::InteractiveMarker &msg ){
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    return marker;
  }

  void timerCallback(const ros::TimerEvent&){
    if (_timer_count == 3){
      sensor_msgs::PointCloud2 manip_cloud_msg;
      pcl::toROSMsg(*_reference_cloud, manip_cloud_msg);
      manip_cloud_msg.header.frame_id = "manipulate_frame";
      manip_cloud_msg.header.stamp = ros::Time::now();
      _pointsPub.publish(manip_cloud_msg);
    }
    if (_timer_count < 25){
      _timer_count++;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"manipulation_data_server");    
  boost::shared_ptr<ManipulationDataServer> server_node(new ManipulationDataServer());
  ManipulationData imk;
  ros::spin();
  server_node.reset();
  return 0;
}
