#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <drc_task_common/InteractiveMarkerArray.h>
#include <drc_task_common/ICPService.h>
#include <jsk_pcl_ros/BoundingBox.h>
#include <jsk_interactive_marker/MarkerDimensions.h>
#include <jsk_interactive_marker/GetTransformableMarkerPose.h>
#include <jsk_interactive_marker/GetMarkerDimensions.h>
#include <jsk_interactive_marker/GetType.h>
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
#include <jsk_pcl_ros/PointsArray.h>
#include <jsk_pcl_ros/ICPAlignWithBox.h>
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

using namespace std;
using namespace ros;
using namespace visualization_msgs;

//int marker array and objects struct
typedef struct Imarkers{
  //marker (maybe bynaly because the date is too big) 
  drc_task_common::InteractiveMarkerArray int_marker_array;
  //point_cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //box for model
  jsk_interactive_marker::MarkerDimensions dim;
  //pose for model
  geometry_msgs::Pose pose;
}IntMarkers;

YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Pose pose){
  out << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::Value;
  out << YAML::Flow << YAML::BeginSeq << pose.position.x << pose.position.y << pose.position.z << YAML::EndSeq;
  out << YAML::Key << "orientation" << YAML::Value;
  out << YAML::Flow << YAML::BeginSeq << pose.orientation.x << pose.orientation.y << pose.orientation.z << pose.orientation.w << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

void operator >> (const YAML::Node& node, geometry_msgs::Pose& pose)
{
  const YAML::Node& position = node["position"];
  position[0] >> pose.position.x;
  position[1] >> pose.position.y;
  position[2] >> pose.position.z;
  const YAML::Node& orientation = node["orientation"];
  orientation[0] >> pose.orientation.x;
  orientation[1] >> pose.orientation.y;
  orientation[2] >> pose.orientation.z;
  orientation[3] >> pose.orientation.w;
}

YAML::Emitter& operator << (YAML::Emitter& out, const jsk_interactive_marker::MarkerDimensions dim){
  out << YAML::BeginMap;
  out << YAML::Key << "dimensions" << YAML::Value;
  out << YAML::Flow << YAML::BeginSeq << dim.x << dim.y << dim.z << dim.radius << dim.small_radius << dim.type << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

void operator >> (const YAML::Node& node, jsk_interactive_marker::MarkerDimensions& dim)
{
  const YAML::Node& dimensions = node["dimensions"];
  dimensions[0] >> dim.x;
  dimensions[1] >> dim.y;
  dimensions[2] >> dim.z;
  dimensions[3] >> dim.radius;
  dimensions[4] >> dim.small_radius;
  dimensions[5] >> dim.type;
}


bool read_marker(const std::string& file_name, IntMarkers& int_markers){
  std::ifstream fin((file_name + string(".yaml")).c_str());
  if (!fin.good()){
    ROS_INFO("Unable to open yaml file", file_name.c_str());
    return false;
  }
  YAML::Parser parser(fin);
  if (!parser) {
    ROS_INFO("Unable to create YAML parser for marker_set");
    return false;
  }
  YAML::Node doc;
  parser.GetNextDocument(doc);
  doc["pose"] >> int_markers.pose;
  doc["dim"] >> int_markers.dim;
  if( pcl::io::loadPCDFile<pcl::PointXYZRGB> (
					      (file_name+string(".pcd")).c_str(), int_markers.cloud) == -1){
    ROS_INFO("couldn't load pcd: %s", file_name.c_str());
    return false;
  }
  std::ifstream ifs((file_name+string(".mkr")).c_str(), std::ios::in|std::ios::binary);
  if(!ifs){
    ROS_INFO("couldn't read marker");
    ifs.close();
  }
  else{
   ifs.seekg (0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg (0, std::ios::beg);
    std::streampos begin = ifs.tellg();
    uint32_t file_size = end-begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read((char*) ibuffer.get(), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, int_markers.int_marker_array);
    ifs.close();
  }
  return true;
}



bool read_marker(int file_index, IntMarkers &imk){
  char file_name[32];
  sprintf(file_name, "sample%02d", file_index);
  return read_marker(std::string(file_name), imk);
}


bool write_marker(const std::string& file_name, IntMarkers &imk){
  std::ofstream out((file_name+std::string(".yaml")).c_str());
  if (!out.is_open())
    {
      ROS_ERROR("Unable to open file [%s] for writing", file_name.c_str());
      return false;
    }
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "pose" << YAML::Value << imk.pose;
  emitter << YAML::Key << "dim" << YAML::Value << imk.dim;
  emitter << YAML::EndMap;
  out << emitter.c_str();
  pcl::io::savePCDFileASCII((file_name+string(".pcd")).c_str(), imk.cloud);
  // Write to File
  std::ofstream ofs((file_name+string(".mkr")).c_str(),  std::ios::out|std::ios::binary);
  uint32_t serial_size = ros::serialization::serializationLength(imk.int_marker_array);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, imk.int_marker_array);
  ofs.write((char*) obuffer.get(), serial_size);
  ofs.close();
  out.close();
  return true;
}

bool write_marker(int file_index, IntMarkers &imk){
  char file_name[32];
  sprintf(file_name, "sample%02d", file_index);
  return write_marker(std::string(file_name), imk);
}

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c) {return sqrt(a * a + b * b + c * c);}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}
inline tf::Transform pose_to_tf(geometry_msgs::Pose pose){
  return tf::Transform(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}
inline geometry_msgs::Pose tf_to_pose(tf::Transform transform){
  geometry_msgs::Pose pose;
  tf::Quaternion q;
  transform.getBasis().getRotation(q);
  pose.orientation.x = q.getX(); pose.orientation.y=q.getY(); pose.orientation.z=q.getZ(), pose.orientation.w=q.getW();
  pose.position.x=transform.getOrigin().getX(), pose.position.y=transform.getOrigin().getY(), pose.position.z=transform.getOrigin().getZ();
  return pose;
}

class PointsNode
{
public:
  tf::TransformBroadcaster br;
  typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
						     jsk_pcl_ros::BoundingBox > SyncPolicy;
  tf::TransformListener listener;
  sensor_msgs::PointCloud2 msg_temp;
  jsk_pcl_ros::BoundingBox msg_box_temp;
protected:
  boost::mutex _mutex;
  ros::NodeHandle _node;
  message_filters::Subscriber <sensor_msgs::PointCloud2> _subPoints;
  message_filters::Subscriber <jsk_pcl_ros::BoundingBox> _subBox;
  Subscriber _subSelectedPoints;
  Subscriber _subSelectedPose;
  Subscriber _sub_pose_feedback;
  Subscriber _sub_object_pose_update;
  Subscriber _sub_object_pose_feedback;
  Publisher _pointsPub;
  Publisher _pointsArrayPub;
  Publisher _debug_cloud_pub;
  Publisher grasp_pose_pub;
  Publisher push_pose_pub;
  Publisher reset_pose_pub;
  Publisher debug_pose_pub;
  Publisher debug_grasp;
  Publisher reset_pub;
  Publisher move_pose_pub;
  Publisher feedback_pub;
  Publisher debug_point_pub;
  Publisher marker_set_pose_pub;
  Publisher t_marker_set_pose_pub;
  ServiceClient client;
  ServiceClient get_type_client;
  ServiceClient get_pose_client;
  ServiceClient get_dim_client;
  ServiceServer align_icp_server;
  ServiceServer save_server;
  ServiceServer assoc_server;
  ServiceServer disassoc_server;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
  tf::Transform tf_from_base;
  tf::Transform tf_before;
  tf::Transform tf_marker;
  tf::Transform tf_from_camera;
  tf::Transform tf_object_constraint;
  ros::Timer tf_timer;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  interactive_markers::MenuHandler menu_handler_first;
  interactive_markers::MenuHandler menu_handler_grasp;
  interactive_markers::MenuHandler menu_handler_push;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_cloud;
  pcl::PointCloud<pcl::Normal>::Ptr reference_cloud_normals;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr reference_kd_tree;
  int reference_num;
  geometry_msgs::Pose menu_pose;
  geometry_msgs::Pose marker_pose;
  geometry_msgs::Pose grasp_pose;
  std::string base_link_name;
  // marker 
  std::vector<boost::shared_ptr<IntMarkers> > markers_array;
  boost::shared_ptr<IntMarkers> markers_ptr;
  // now reference marker's ptr ;
  int timer_count;
  bool reference_hit;
  bool assoc_marker_flug_;
  //bool init_reference_end;
public:
  bool transformPointcloudInBoundingBox(
					const jsk_pcl_ros::BoundingBox& box_msg,
    const sensor_msgs::PointCloud2& cloud_msg,
    pcl::PointCloud<pcl::PointXYZRGB>& output,
    Eigen::Affine3f& offset,
    tf::TransformListener& tf_listener)
  {
    geometry_msgs::PoseStamped box_pose;
    box_pose.header = box_msg.header;
    box_pose.pose = box_msg.pose;
    // transform box_pose into msg frame
    geometry_msgs::PoseStamped box_pose_respected_to_cloud;
    tf_listener.transformPose(cloud_msg.header.frame_id,
                                box_pose,
                                box_pose_respected_to_cloud);
    // convert the pose into eigen
    Eigen::Affine3d box_pose_respected_to_cloud_eigend;
    tf::poseMsgToEigen(box_pose_respected_to_cloud.pose,
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


  PointsNode():timer_count(100),tf_timer(_node.createTimer(ros::Duration(0.01), &PointsNode::timerCallback, this)), reference_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), reference_cloud_normals(new pcl::PointCloud<pcl::Normal>), reference_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
  {
    //init_reference_end=false;
    init_reference();
    assoc_marker_flug_=false;
    ROS_INFO("start interface");
    ros::NodeHandle local_nh("~");
    local_nh.param("BASE_FRAME_ID", base_link_name, std::string("/camera_link"));
    ROS_INFO("base_link_name: %s", base_link_name.c_str());
    server.reset(new interactive_markers::InteractiveMarkerServer("manip","",false) );
    menu_handler_first.insert("Grasp", boost::bind(&PointsNode::grasp_cb, this, _1));
    menu_handler_first.insert("Push", boost::bind(&PointsNode::push_cb, this, _1));
    menu_handler_first.insert("Remove", boost::bind(&PointsNode::remove_cb, this, _1));
    menu_handler_first.insert("Reset Pose", boost::bind(&PointsNode::reset_pose_cb, this, _1));
    menu_handler_grasp.insert("do_grasp", boost::bind(&PointsNode::do_grasp_cb, this, _1));   
    menu_handler_grasp.insert("Remove", boost::bind(&PointsNode::remove_cb, this, _1));
    menu_handler_grasp.insert("Reset Pose", boost::bind(&PointsNode::reset_pose_cb, this, _1));
    menu_handler_push.insert("do_push", boost::bind(&PointsNode::do_push_cb, this, _1));
    menu_handler_push.insert("Remove", boost::bind(&PointsNode::remove_cb, this, _1));
    menu_handler_push.insert("Reset Pose", boost::bind(&PointsNode::reset_pose_cb, this, _1));
    _subPoints.subscribe(_node , "/selected_pointcloud", 1);
    _pointsPub = _node.advertise<sensor_msgs::PointCloud2>("/manip_points", 10);
    _pointsArrayPub = _node.advertise<sensor_msgs::PointCloud2>("/icp_registration/input_reference_add", 10);
    _debug_cloud_pub = _node.advertise<sensor_msgs::PointCloud2>("/manip/debug_cloud", 10);
    grasp_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/grasp_pose", 10);
    push_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/push_pose", 10);
    debug_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/debug_pose", 10);
    debug_point_pub = _node.advertise<geometry_msgs::PointStamped>("/debug_point", 10);
    reset_pose_pub = _node.advertise<std_msgs::String>("/reset_pose_command", 1);
    reset_pub = _node.advertise<jsk_pcl_ros::PointsArray>("/icp_registration/input_reference_array", 1, boost::bind( &PointsNode::icp_connection, this, _1), boost::bind( &PointsNode::icp_disconnection, this, _1));
    move_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/move_pose", 1);
    feedback_pub = _node.advertise<visualization_msgs::InteractiveMarkerFeedback>("/interactive_point_cloud/feedback", 1);
    usleep(100000);
    debug_grasp = _node.advertise<visualization_msgs::Marker>("/debug_grasp", 1);
    marker_set_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/interactive_point_cloud/set_marker_pose", 1);
    t_marker_set_pose_pub = _node.advertise<geometry_msgs::PoseStamped>("/transformable_interactive_server/set_pose", 1);
    _subBox.subscribe(_node, "/bounding_box_marker/selected_box", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
   
    sync_->connectInput(_subPoints, _subBox);
    sync_->registerCallback(boost::bind(
					&PointsNode::set_reference,
					this, _1, _2));
    client = _node.serviceClient<jsk_pcl_ros::ICPAlignWithBox>("/icp_registration/icp_service");
    get_type_client = _node.serviceClient<jsk_interactive_marker::GetType>("/transformable_interactive_server/get_type");
    get_pose_client = _node.serviceClient<jsk_interactive_marker::GetTransformableMarkerPose>("/transformable_interactive_server/get_pose");
    get_dim_client = _node.serviceClient<jsk_interactive_marker::GetMarkerDimensions>("/transformable_interactive_server/get_dimensions");

    _subSelectedPoints = _node.subscribe("/selected_points",1,&PointsNode::set_menu_cloud,this);
    _subSelectedPose = _node.subscribe("/interactive_point_cloud/left_click_point_relative", 1, &PointsNode::set_menu_point, this);
    _sub_pose_feedback = _node.subscribe("/interactive_point_cloud/feedback", 1, &PointsNode::marker_move_feedback, this);
    _sub_object_pose_update = _node.subscribe("/simple_marker/update", 1, &PointsNode::t_marker_move_update, this);
    _sub_object_pose_feedback = _node.subscribe("/simple_marker/feedback", 1, &PointsNode::t_marker_move_feedback, this);
    align_icp_server = _node.advertiseService("icp_apply", &PointsNode::align_cb, this);
    save_server = _node.advertiseService("save_manipulation", &PointsNode::save_cb, this);
    assoc_server = _node.advertiseService("assoc_points", &PointsNode::assoc_object_to_marker_cb, this);
    disassoc_server = _node.advertiseService("disassoc_points", &PointsNode::disassoc_object_to_marker_cb, this);
    tf_from_base.setOrigin(tf::Vector3(0, 0, 0));
    tf_from_base.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_marker = tf_before = tf_from_base;
    //
    // ros::Rate poll_rate(100);
    // ROS_INFO("num sub %d", reset_pub.getNumSubscribers());
    // while(reset_pub.getNumSubscribers() == 0){
    //   poll_rate.sleep();
    // }
    // pub_reference();
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
      boost::shared_ptr<Imarkers> temp_marker(new Imarkers);
      if(read_marker(pcd_files[i].c_str(), *temp_marker)){
	markers_array.push_back(temp_marker);
      }
    }
  }
  void pub_reference(){
    jsk_pcl_ros::PointsArray reset_points;
    pcl::PointCloud <pcl::PointXYZRGB> all_points;
    for(size_t i=0; i<markers_array.size(); i++){
      boost::shared_ptr<Imarkers> temp_marker = markers_array[i];
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
    reset_pub.publish(reset_points);
    ROS_INFO("initialize done");
  }
  bool save_cb(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
  {
    save_marker();
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*reference_cloud, ros_output);
    ros_output.header.stamp = ros::Time::now();
    ros_output.header.frame_id = "manipulate_frame";
    _debug_cloud_pub.publish(ros_output);
    _pointsPub.publish(ros_output);
    if(!reference_hit){
      _pointsArrayPub.publish(ros_output);
      reference_hit = true;
      markers_array.push_back(markers_ptr);
    }
    else{
      ROS_INFO("before cloud reused");
    }
    return true;
  }
  bool save_marker(){
    if(markers_ptr){
      InteractiveMarker int_marker_tmp;
      markers_ptr->int_marker_array.int_markers.resize(0);
      if(server->get("first_menu", int_marker_tmp)){
	markers_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
	server->erase("first_menu");
      }
      if(server->get("grasp_pose", int_marker_tmp)){
	markers_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
	server->erase("grasp_pose");
      }
      if(server->get("push_pose", int_marker_tmp)){
	markers_ptr->int_marker_array.int_markers.push_back(int_marker_tmp);
	server->erase("push_pose");
      }
      server->applyChanges();
      markers_ptr->cloud = *reference_cloud;
      jsk_interactive_marker::GetMarkerDimensions get_dim_srv;
      jsk_interactive_marker::GetTransformableMarkerPose get_pose_srv;
      jsk_interactive_marker::GetType get_type_srv;
      get_dim_srv.request.target_name = get_pose_srv.request.target_name = "";
      if (get_dim_client.call(get_dim_srv)){markers_ptr->dim=get_dim_srv.response.dimensions;}
      if (get_type_client.call(get_type_srv)){markers_ptr->dim.type=get_type_srv.response.type;}
      if (get_pose_client.call(get_pose_srv)){
        ROS_INFO("pose service succeed");
	geometry_msgs::PoseStamped temp_pose_stamped;
	listener.transformPose("manipulate_frame",ros::Time::now()-(ros::Duration(0.2)), get_pose_srv.response.pose_stamped,get_pose_srv.response.pose_stamped.header.frame_id, temp_pose_stamped);
	markers_ptr->pose=temp_pose_stamped.pose;
	ROS_INFO("save_marker_pose: %f %f %f" ,temp_pose_stamped.pose.position.x, temp_pose_stamped.pose.position.y, temp_pose_stamped.pose.position.z, 
                 temp_pose_stamped.pose.orientation.x, temp_pose_stamped.pose.orientation.y, temp_pose_stamped.pose.orientation.z, temp_pose_stamped.pose.orientation.w);
      }
      else{
        ROS_INFO("save failed");
	markers_ptr->pose.orientation.x = 0;
	markers_ptr->pose.orientation.y = 0;
        markers_ptr->pose.orientation.z = 0;
        markers_ptr->pose.orientation.w = 1;
      }
      // service call current pose and box
      // change pose and box to manipulate_frame
      write_marker(reference_num, *markers_ptr);
      ROS_INFO("save succeeded %d", reference_num);
      return true;
    }else{
      return false;
    }
  }
  void icp_connection(const ros::SingleSubscriberPublisher& pub){
    pub_reference();
    ROS_INFO("connection abled");
  }
  void icp_disconnection(const ros::SingleSubscriberPublisher& pub){  
    ROS_INFO("connection disabled");
  }
  ~PointsNode() {
  }
  void t_marker_move_update(const visualization_msgs::InteractiveMarkerUpdate update)
  {
    if(!assoc_marker_flug_) return;
    if(update.type == 0) return;
    if(update.markers.size()==1) {
      tf::Transform tf_to_object = pose_to_tf(update.markers[0].pose);
      geometry_msgs::PoseStamped marker_pose_stamped;
      marker_pose_stamped.pose = tf_to_pose( tf_to_object * tf_object_constraint.inverse());
      marker_pose_stamped.header = update.markers[0].header;
      marker_set_pose_pub.publish(marker_pose_stamped);
      geometry_msgs::PoseStamped marker_pose_stamped_from_manip;
      listener.transformPose("/manipulate_frame",ros::Time::now()-(ros::Duration(0.2)), marker_pose_stamped , marker_pose_stamped.header.frame_id, marker_pose_stamped_from_manip);
      tf_marker=pose_to_tf(marker_pose_stamped_from_manip.pose);

    }
  }
  void t_marker_move_feedback(const visualization_msgs::InteractiveMarkerFeedback feedback)
  {
    if(!assoc_marker_flug_) return;
    if(feedback.event_type == 1) {
      tf::Transform tf_to_object = pose_to_tf(feedback.pose);
      geometry_msgs::PoseStamped marker_pose_stamped;
      marker_pose_stamped.pose = tf_to_pose( tf_to_object * tf_object_constraint.inverse());
      marker_pose_stamped.header = feedback.header;
      marker_set_pose_pub.publish(marker_pose_stamped);
      geometry_msgs::PoseStamped marker_pose_stamped_from_manip;
      listener.transformPose("/manipulate_frame",ros::Time::now()-(ros::Duration(0.2)), marker_pose_stamped , marker_pose_stamped.header.frame_id, marker_pose_stamped_from_manip);
      tf_marker=pose_to_tf(marker_pose_stamped_from_manip.pose);

    }
  }
  void marker_move_feedback(const visualization_msgs::InteractiveMarkerFeedback feedback)
  {
    //
    geometry_msgs::PoseStamped temp_pose_stamped, temp_feedback_pose_stamped;
    temp_feedback_pose_stamped.pose = feedback.pose;
    temp_feedback_pose_stamped.header = feedback.header;
    try{
      listener.transformPose("manipulate_frame",ros::Time::now()-(ros::Duration(0.2)), temp_feedback_pose_stamped , feedback.header.frame_id, temp_pose_stamped);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("marker move failed %s",ex.what());
      return;
    }

    marker_pose = temp_pose_stamped.pose;
    //ROS_INFO("feedback_header %s", feedback.header.frame_id.c_str());
    //listener.transformPose("manipulate_frame", feedback.pose,marker_pose);
    tf::poseMsgToTF(marker_pose, tf_marker);
    //depends jsk_interactive_marker
    if(assoc_marker_flug_){
      geometry_msgs::PoseStamped t_pose_stamped;
      t_pose_stamped.pose = tf_to_pose(tf_marker*tf_object_constraint);
      t_pose_stamped.header = temp_pose_stamped.header;
      t_marker_set_pose_pub.publish(t_pose_stamped);
    }
    if(feedback.menu_entry_id==1){
      geometry_msgs::PoseStamped move_pose;
      move_pose.header = feedback.header;
      move_pose.header.frame_id=std::string("marker_frame");
      move_pose.pose = grasp_pose;
      move_pose_pub.publish(move_pose);
      //pub zero feedback
      marker_pose.position.x=marker_pose.position.y=marker_pose.position.z=0;
      marker_pose.orientation.x=marker_pose.orientation.y=marker_pose.orientation.z=0;marker_pose.orientation.w=1;
      // visualization_msgs::InteractiveMarkerFeedback feedback_temp = feedback;
      // feedback_temp.menu_entry_id=0;
      // feedback_temp.pose = marker_pose;
      // feedback_temp.header.stamp=ros::Time::now();
      // feedback_pub.publish(feedback_tp);
      //move manip frame
      sensor_msgs::PointCloud2 manip_cloud_msg;
      pcl::toROSMsg(*reference_cloud, manip_cloud_msg);
      manip_cloud_msg.header = feedback.header;
      _pointsPub.publish(manip_cloud_msg);
      tf_before = tf_from_base;
      tf_from_base = tf_before * tf_marker;
      tf::poseMsgToTF(marker_pose, tf_marker);
      timer_count = 0;
    }

  }
  void reset_pose_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    std_msgs::String a;
    reset_pose_pub.publish(a);
  }
  void push_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("push_cb driven");
    //remove circle marker
    server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.14;
    int_marker.name = "push_pose";
    int_marker.pose = menu_pose;
    visualization_msgs::InteractiveMarkerControl push_control;
    push_control.always_visible = true;
    visualization_msgs::Marker arrow_marker;
    arrow_marker.type = Marker::ARROW;
    arrow_marker.scale.x = 0.11;
    arrow_marker.scale.y = 0.04;
    arrow_marker.scale.z = 0.04;
    arrow_marker.color.r = .8; arrow_marker.color.g = .8; arrow_marker.color.b = .8; arrow_marker.color.a = 1.0;
    arrow_marker.pose.position.x = -.11;
    arrow_marker.pose.position.y = 0.0;
    arrow_marker.pose.position.z = 0.0;
    push_control.markers.push_back(arrow_marker);
    
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
    server->insert(int_marker);
    menu_handler_push.apply( *server , "push_pose");
    server->applyChanges();

    // Todo
  }
  void remove_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("remove_cb driven");
    server->erase(feedback->marker_name);
    server->applyChanges();
  }
  void do_grasp_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = feedback->header;
    pose_msg.header.frame_id = std::string("manipulate_frame");
    pose_msg.pose = feedback->pose;
    grasp_pose_pub.publish(pose_msg);
    // mode change 
    grasp_pose = feedback->pose;
  }
  void do_push_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = feedback->header;
    pose_msg.header.frame_id = std::string("manipulate_frame");
    pose_msg.pose = feedback->pose;
    push_pose_pub.publish(pose_msg);
  }
  void grasp_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    ROS_INFO("grasp_cb driven");
    //remove circle marker
    server->erase("first_menu");
    //add grasp marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = feedback->header.frame_id;
    int_marker.scale = 0.2;
    int_marker.name = "grasp_pose";
    int_marker.pose = menu_pose;
    visualization_msgs::InteractiveMarkerControl grab_control;
    grab_control.always_visible = true;
    grab_control.markers.push_back(makegrab(int_marker.scale*.6, int_marker.scale*.3, int_marker.scale*.3,
				       0, 0, 1,
				       int_marker.scale*.3, int_marker.scale*.3, 0
				      ));
    grab_control.markers.push_back(makegrab(int_marker.scale*.6, int_marker.scale*.3, int_marker.scale*.3,
				      1, 0, 0,
				       int_marker.scale*.3, -int_marker.scale*.3, 0
				      ));
    grab_control.markers.push_back(makegrab(int_marker.scale*.3, int_marker.scale*.9, int_marker.scale*.3,
				       0, 1, 0,
				      int_marker.scale*-.15, 0, 0
				       ));
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
  
    server->insert(int_marker);
    menu_handler_grasp.apply( *server , "grasp_pose");
    server->applyChanges();
  }
  bool align_cb(drc_task_common::ICPService::Request& req,
		drc_task_common::ICPService::Response& res)
  {
    set_icp((req.points), (req.box));
    if(reference_hit){
      res.dim = markers_ptr->dim;
      geometry_msgs::PoseStamped temp_pose_stamped;
      geometry_msgs::Pose temp_pose, marker_pose = markers_ptr->pose;
      ROS_INFO("marker_pose, %f %f %f %f %f %f %f", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z, marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w );
      tf::Transform tf_transformable_marker(tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w), tf::Vector3(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z));
      tf::Transform tf_marker_to_camera = tf_from_camera * tf_transformable_marker;
      temp_pose.position.x = tf_marker_to_camera.getOrigin().getX();
      temp_pose.position.y = tf_marker_to_camera.getOrigin().getY();
      temp_pose.position.z = tf_marker_to_camera.getOrigin().getZ();
      tf::Quaternion temp_qua;
      tf_marker_to_camera.getBasis().getRotation(temp_qua);
      temp_pose.orientation.x = temp_qua.getX();
      temp_pose.orientation.y = temp_qua.getY();
      temp_pose.orientation.z = temp_qua.getZ();
      temp_pose.orientation.w = temp_qua.getW();
      temp_pose_stamped.header = req.points.header;
      temp_pose_stamped.pose = temp_pose;
      debug_pose_pub.publish(temp_pose_stamped);
      res.pose_stamped = temp_pose_stamped;
      return true;
    }else{
      return false;
    }
  }
  bool assoc_object_to_marker_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    return assoc_object_to_marker();
  }
  bool assoc_object_to_marker(){
    assoc_marker_flug_=true;
    if(assoc_marker_flug_){
      jsk_interactive_marker::GetTransformableMarkerPose get_pose_srv;
      get_pose_srv.request.target_name="";
      if(get_pose_client.call(get_pose_srv)){
	geometry_msgs::PoseStamped after_pose_;
	listener.transformPose("marker_frame",ros::Time::now()-(ros::Duration(0.2)), get_pose_srv.response.pose_stamped, get_pose_srv.response.pose_stamped.header.frame_id, after_pose_);
	tf_object_constraint = pose_to_tf(after_pose_.pose);
      }
    }
    return true;
  }
  
  bool disassoc_object_to_marker_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    return disassoc_object_to_marker();
  }
  bool disassoc_object_to_marker(){
    assoc_marker_flug_=false;
    return true;
  }
  void set_reference(const sensor_msgs::PointCloud2ConstPtr& msg_ptr, const jsk_pcl_ros::BoundingBoxConstPtr& box_ptr)
  {
    sensor_msgs::PointCloud2 msg = *msg_ptr;
    jsk_pcl_ros::BoundingBox box = *box_ptr;
    set_icp(msg, box);
    if(reference_hit){
      //res.dim = markers_ptr->dim;
      geometry_msgs::PoseStamped temp_pose_stamped;
      geometry_msgs::Pose temp_pose, marker_pose = markers_ptr->pose;
      tf::Transform tf_transformable_marker(tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w), tf::Vector3(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z));
      ROS_INFO("marker_pose:%f %f %f", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
      tf::Transform tf_marker_to_camera = tf_from_camera * tf_transformable_marker;
      temp_pose.position.x = tf_marker_to_camera.getOrigin().getX();
      temp_pose.position.y = tf_marker_to_camera.getOrigin().getY();
      temp_pose.position.z = tf_marker_to_camera.getOrigin().getZ();
      tf::Quaternion temp_qua;
      tf_marker_to_camera.getBasis().getRotation(temp_qua);
      temp_pose.orientation.x = temp_qua.getX();
      temp_pose.orientation.y = temp_qua.getY();
      temp_pose.orientation.z = temp_qua.getZ();
      temp_pose.orientation.w = temp_qua.getW();
      temp_pose_stamped.header = msg.header;
      temp_pose_stamped.pose = temp_pose;
      debug_pose_pub.publish(temp_pose_stamped);
      //res.pose_stamped = temp_pose_stamped;
      //return true;
    }else{
      //return false;
    }

  }
  void set_icp(sensor_msgs::PointCloud2& msg, jsk_pcl_ros::BoundingBox& box){
    sensor_msgs::PointCloud2* msg_ptr = &msg;
    jsk_pcl_ros::BoundingBox* box_ptr = &box;
    //believes that header is same
    //maybe check diference, if there are none, box_ptr will be selected 
    //marker_init
    geometry_msgs::PoseStamped before_pose_, after_pose_;
    jsk_pcl_ros::ICPAlignWithBox srv; 
    disassoc_object_to_marker();
    srv.request.target_cloud = *msg_ptr;
    srv.request.target_box = *box_ptr;
    //save_marker();
    if (client.call(srv) && srv.response.result.score < 0.000075){
      before_pose_.pose = srv.response.result.pose;
      before_pose_.header = srv.response.result.header;
      ROS_INFO("before pose driven");
      ROS_INFO("score was %f", srv.response.result.score);
      ROS_INFO("best reference %s", srv.response.result.name.c_str());
      int num  = atoi(srv.response.result.name.c_str());
      ROS_INFO("reference_num was %d", num);
      reference_num = num;
      boost::shared_ptr<IntMarkers> markers = markers_array[num];
      markers_ptr = markers_array[num];
      for(std::vector<InteractiveMarker>::iterator marker_one_ptr = markers->int_marker_array.int_markers.begin() ; marker_one_ptr!= markers->int_marker_array.int_markers.end() ;marker_one_ptr++){
	server->insert(*marker_one_ptr);
      }
      menu_handler_first.apply( *server , "first_menu");
      menu_handler_grasp.apply( *server , "grasp_pose");
      menu_handler_push.apply( *server , "push_pose");
      server->applyChanges();
      reference_hit=true;      
    }
    else {
      before_pose_.pose = box_ptr->pose;
      before_pose_.header = box_ptr->header;
      ROS_INFO("new pose");
      markers_ptr = boost::make_shared<IntMarkers> ();
      //markers_array.push_back(markers_ptr);
      reference_num = markers_array.size() - 1;
      reference_hit=false;
    }
    listener.transformPose(base_link_name,ros::Time::now()-(ros::Duration(0.2)), before_pose_, before_pose_.header.frame_id, after_pose_);
    // listener.waitForTransform(base_link_name, before_pose_.header.frame_id, ros::Time(0), ros::Duration(10.0));
    // listener.transformPose(base_link_name, before_pose_,after_pose_);
    tf_before = tf_from_base;
    msg_temp = *msg_ptr;
    msg_box_temp = *box_ptr;
    tf_from_base.setOrigin(tf::Vector3(after_pose_.pose.position.x,after_pose_.pose.position.y, after_pose_.pose.position.z));
    tf_from_base.setRotation(tf::Quaternion(after_pose_.pose.orientation.x, after_pose_.pose.orientation.y, after_pose_.pose.orientation.z, after_pose_.pose.orientation.w));
    tf_from_camera.setOrigin(tf::Vector3(before_pose_.pose.position.x,before_pose_.pose.position.y, before_pose_.pose.position.z));
    tf_from_camera.setRotation(tf::Quaternion(before_pose_.pose.orientation.x, before_pose_.pose.orientation.y, before_pose_.pose.orientation.z, before_pose_.pose.orientation.w));


    tf_from_base = tf_from_base;//*transform_camera_to_optical;
    tf_marker = tf::Transform(tf::Quaternion(0, 0, 0, 1));
    timer_count = 0; 
    //move point_cloud
    try
      {
	Eigen::Affine3f offset;
	transformPointcloudInBoundingBox(
   			      *box_ptr, *msg_ptr,
    			      *reference_cloud, offset,
			      listener);
	Eigen::Affine3f offset_inverse = offset.inverse();
      
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(reference_cloud);
	ne.setSearchMethod(reference_kd_tree);
	ne.setRadiusSearch(0.02);
	ne.setViewPoint(offset.translation().x(), offset.translation().y(), offset.translation().z());//after_orig.point.x, after_orig.point.y, after_orig.point.z);
	ne.compute(*reference_cloud_normals);
	pcl::PointCloud<pcl::PointXYZRGBNormal> concatenated_cloud;
	concatenated_cloud.points.resize(reference_cloud->points.size());
	concatenated_cloud.width = reference_cloud->width;
	concatenated_cloud.height = reference_cloud->height;
	concatenated_cloud.is_dense = reference_cloud->is_dense;
	
	for (size_t i = 0; i < concatenated_cloud.points.size(); i++) {
	  pcl::PointXYZRGBNormal point;
	  point.x = reference_cloud->points[i].x;
	  point.y = reference_cloud->points[i].y;
	  point.z = reference_cloud->points[i].z;
	  point.rgb = reference_cloud->points[i].rgb;
	  point.normal_x = reference_cloud_normals->points[i].normal_x;
	  point.normal_y = reference_cloud_normals->points[i].normal_y;
	  point.normal_z = reference_cloud_normals->points[i].normal_z;
	  point.curvature = reference_cloud_normals->points[i].curvature;
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
    
  }
  
  void set_menu(float p_x, float p_y, float p_z, std_msgs::Header header){
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = p_x;
    searchPoint.y = p_y;
    searchPoint.z = p_z;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (reference_kd_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    }
    else{
      ROS_INFO("kdtree failed");
      return;
    }
    
    float x = menu_pose.position.x=reference_cloud->points[pointIdxNKNSearch[0]].x;
    float y = menu_pose.position.y=reference_cloud->points[pointIdxNKNSearch[0]].y; 
    float z = menu_pose.position.z=reference_cloud->points[pointIdxNKNSearch[0]].z; 
    float v_x= reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_x;
    float v_y= reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_y;
    float v_z= reference_cloud_normals->points[pointIdxNKNSearch[0]].normal_z;
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
      ROS_INFO("affine_orig, %f %f %f", x, y, z);

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
      pcl::transformPointCloud(*reference_cloud, *output_cloud, offset);

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
	ROS_INFO("rad=%f, width=%f", theta_,before_w);
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
    
    debug_grasp.publish(hand_marker);
    menu_pose.orientation.x= min_qua.getX();
    menu_pose.orientation.y= min_qua.getY();
    menu_pose.orientation.z= min_qua.getZ();
    menu_pose.orientation.w= min_qua.getW();
    
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/marker_frame";
    int_marker.scale = 0.2;
    int_marker.name = "first_menu";

    int_marker.pose = menu_pose;
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.markers.push_back( makeCircle(int_marker ));
    int_marker.controls.push_back(control);
    
    server->insert( int_marker );
    menu_handler_first.apply( *server , "first_menu");
    server->applyChanges();    

  }
  void set_menu_point(const geometry_msgs::PointStampedPtr & msg_ptr)
  {
    ROS_INFO("points selected with marker");
    geometry_msgs::PointStamped after_point;
    ROS_INFO("before_set_menu");
    //listener.transformPoint("/manipulate_frame", ros::Time::now() - (ros::Duration(0.05)) ,*msg_ptr, msg_ptr->header.frame_id,  after_point);
    after_point = *msg_ptr;
    ROS_INFO("after_set_menu");
    set_menu(after_point.point.x, after_point.point.y, after_point.point.z, msg_ptr->header);
    pcl::PointXYZRGB searchPoint;
  }
  
  void set_menu_cloud(const sensor_msgs::PointCloud2ConstPtr& msg_ptr) // may be marker_
  { 
    ROS_INFO("points selected");
    // change frame
    if(reference_cloud->points.size()==0){
      ROS_INFO("points is empty");
      return;
    }
    sensor_msgs::PointCloud before_cloud;
    sensor_msgs::PointCloud after_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg_ptr, before_cloud);
    br.sendTransform(tf::StampedTransform(tf_from_base, ros::Time::now(), base_link_name, "marker_frame"));
    listener.transformPointCloud("/marker_frame", before_cloud, after_cloud);
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
  visualization_msgs::Marker makegrab(float s_x, float s_y, float s_z, float r, float g, float b, float x, float y, float z)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    return marker;
  }


  void timerCallback(const ros::TimerEvent&){
    ros::Time a = ros::Time::now() - (ros::Duration(0.05));
    br.sendTransform(tf::StampedTransform(tf_calc(), ros::Time::now(), base_link_name, "manipulate_frame"));
    br.sendTransform(tf::StampedTransform(tf_calc()*tf_marker, ros::Time::now(), base_link_name, "marker_frame"));
    if (timer_count == 30){
      sensor_msgs::PointCloud2 manip_cloud_msg;
      pcl::toROSMsg(*reference_cloud, manip_cloud_msg);
      manip_cloud_msg.header.frame_id = "manipulate_frame";
      manip_cloud_msg.header.stamp = ros::Time::now();
      _pointsPub.publish(manip_cloud_msg);
    }
    if (timer_count < 250){
      timer_count++;
    }
  }
  tf::Transform tf_calc()
  {
    if (timer_count > 20){
      return tf_from_base;
    }
    tf::Vector3 vec = tf_from_base.getOrigin()*(1-timer_count/20.0) + tf_before.getOrigin()*timer_count/20.0;
    tf::Quaternion qua_b, qua_a;
    tf_before.getBasis().getRotation(qua_b);
    tf_from_base.getBasis().getRotation(qua_a);
    tf::Quaternion qua = qua_b.slerp(qua_a, timer_count/10.0);
    return tf::Transform(qua, vec);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"points_and_menu");    
  boost::shared_ptr<PointsNode> pointsnode(new PointsNode());
  Imarkers imk;
  ros::spin();
  // tf_change()
  pointsnode.reset();
  return 0;
}
