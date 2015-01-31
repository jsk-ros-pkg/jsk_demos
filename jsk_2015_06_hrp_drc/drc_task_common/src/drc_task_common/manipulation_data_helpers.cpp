#include <drc_task_common/manipulation_data_helpers.h>
#include <ros/node_handle.h>

namespace manip_helpers
{
//int marker array and objects struct

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
#ifdef USE_OLD_YAML
  position[0] >> pose.position.x;
  position[1] >> pose.position.y;
  position[2] >> pose.position.z;
#else
  pose.position.x = position[0].as<double>();
  pose.position.y = position[1].as<double>();
  pose.position.z = position[2].as<double>();
#endif
  const YAML::Node& orientation = node["orientation"];
#ifdef USE_OLD_YAML
  orientation[0] >> pose.orientation.x;
  orientation[1] >> pose.orientation.y;
  orientation[2] >> pose.orientation.z;
  orientation[3] >> pose.orientation.w;
#else
  pose.orientation.x = orientation[0].as<double>();
  pose.orientation.y = orientation[1].as<double>();
  pose.orientation.z = orientation[2].as<double>();
  pose.orientation.w = orientation[3].as<double>();
#endif
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
#ifdef USE_OLD_YAML
  dimensions[0] >> dim.x;
  dimensions[1] >> dim.y;
  dimensions[2] >> dim.z;
  dimensions[3] >> dim.radius;
  dimensions[4] >> dim.small_radius;
  dimensions[5] >> dim.type;
#else
  dim.x = dimensions[0].as<float>();
  dim.y = dimensions[1].as<float>();
  dim.z = dimensions[2].as<float>();
  dim.radius = dimensions[3].as<float>();
  dim.small_radius = dimensions[4].as<float>();
  dim.type = dimensions[5].as<float>();
#endif
}


bool read_marker(const std::string& file_name, ManipulationData& int_markers){
  YAML::Node doc;
#ifdef USE_OLD_YAML
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
  parser.GetNextDocument(doc);
#else
  // yaml-cpp is greater than 0.5.0
  doc = YAML::LoadFile(file_name + string(".yaml"));
#endif
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



bool read_marker(int file_index, ManipulationData &imk){
  char file_name[32];
  sprintf(file_name, "sample%02d", file_index);
  return read_marker(std::string(file_name), imk);
}


bool write_marker(const std::string& file_name, ManipulationData &imk){
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

bool write_marker(int file_index, ManipulationData &imk){
  char file_name[32];
  sprintf(file_name, "sample%02d", file_index);
  return write_marker(std::string(file_name), imk);
}

std::vector<visualization_msgs::Marker> make_grab(visualization_msgs::InteractiveMarker &int_marker, float brightness)
{
  std::vector<visualization_msgs::Marker> markers;
  markers.push_back(make_box(int_marker.scale*.6, int_marker.scale*.3, int_marker.scale*.3,
			     0, 0, 1*brightness,
			     int_marker.scale*.3, int_marker.scale*.3, 0
			     ));
  markers.push_back(make_box(int_marker.scale*.6, int_marker.scale*.3, int_marker.scale*.3,
			     1*brightness, 0, 0,
			     int_marker.scale*.3, -int_marker.scale*.3, 0
			     ));
  markers.push_back(make_box(int_marker.scale*.3, int_marker.scale*.9, int_marker.scale*.3,
			     0, 1*brightness, 0,
			     int_marker.scale*-.15, 0, 0
			     ));
  return markers;
}
visualization_msgs::Marker make_box(float s_x, float s_y, float s_z, float r, float g, float b, float x, float y, float z)
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
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1.0;
  return marker;
}
  
visualization_msgs::Marker make_circle(visualization_msgs::InteractiveMarker &msg ){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  return marker;
}
visualization_msgs::Marker make_arrow(visualization_msgs::InteractiveMarker &msg, float arrow_offset_x_pos, float r, float g, float b, float a, float arrow_length){
  visualization_msgs::Marker marker;
  marker.type = Marker::ARROW;
  marker.scale.x = arrow_length;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = a;
  marker.pose.position.x = arrow_offset_x_pos;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  return marker;
}

std::string make_name(std::string name, int id){
  char num_string[16];
  sprintf(num_string, "_%d", id);
  return name + std::string(num_string);
}
void name_marker(Marker *marker, std::string name, int id){
  marker->id = id;
  marker->ns = make_name(name, id);
}
tf::Transform pose_to_tf(geometry_msgs::Pose pose){
  return tf::Transform(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}
geometry_msgs::Pose tf_to_pose(tf::Transform transform){
  geometry_msgs::Pose pose;
  tf::Quaternion q;
  transform.getBasis().getRotation(q);
  pose.orientation.x = q.getX(); pose.orientation.y=q.getY(); pose.orientation.z=q.getZ(), pose.orientation.w=q.getW();
  pose.position.x=transform.getOrigin().getX(), pose.position.y=transform.getOrigin().getY(), pose.position.z=transform.getOrigin().getZ();
  return pose;
}
geometry_msgs::Pose change_pose(geometry_msgs::Pose base_pose, geometry_msgs::Pose child_pose){
  return tf_to_pose(pose_to_tf(base_pose)*pose_to_tf(child_pose));
}

visualization_msgs::MarkerArray make_array(ManipulationData manip_data, double x_pos, double y_pos, int id, std::string name, std::string frame_id_name){
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = name;
  marker.header.stamp = ros::Time::now();
  // marker.ns = name;
  // marker.id = id;
  int id_index = id*100;
  marker.pose.position.x = x_pos;
  marker.pose.position.y = y_pos;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
  // for int_marker
  for(size_t i=0; i<manip_data.int_marker_array.int_markers.size(); i++){
    for(size_t j=0; j<manip_data.int_marker_array.int_markers[i].controls.size();j++){
      for(size_t k=0; k<manip_data.int_marker_array.int_markers[i].controls[j].markers.size();k++){ 
	Marker marker_temp = manip_data.int_marker_array.int_markers[i].controls[j].markers[k];
	name_marker(&marker_temp, name, id_index++);
	marker_temp.header = marker.header;
	if(marker_temp.pose.orientation.x==0.0&&marker_temp.pose.orientation.y==0.0&&marker_temp.pose.orientation.x==0.0&&marker_temp.pose.orientation.w==0.0)
	  marker_temp.pose.orientation.w=1.0;
	marker_temp.pose = change_pose(manip_data.int_marker_array.int_markers[i].pose, marker_temp.pose);
	marker_temp.pose.position.x+=x_pos; marker_temp.pose.position.y+=y_pos;
	marker_array.markers.push_back(marker_temp);
      }
    } 
  }
  // for points
  double size = 0.004;
  name_marker(&marker, name, id_index++);
  marker.scale.x = marker.scale.y = marker.scale.x = size;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;  
  size_t num_points = manip_data.cloud.points.size();
  marker.points.resize(num_points);
  marker.colors.resize(num_points);
  for (size_t i=0; i<num_points; i++){
    marker.points[i].x = manip_data.cloud.points[i].x;
    marker.points[i].y = manip_data.cloud.points[i].y;
    marker.points[i].z = manip_data.cloud.points[i].z;
    marker.colors[i].r = manip_data.cloud.points[i].r/255.;
    marker.colors[i].g = manip_data.cloud.points[i].g/255.;
    marker.colors[i].b = manip_data.cloud.points[i].b/255.;
    marker.colors[i].a = 1.0;
  }
  marker_array.markers.push_back(marker);
  marker.points.resize(0);
  marker.colors.resize(0);
  // for t-object
  // name_marker(&marker, name, id_index++);
  // marker_array.marker.push_back(marker);
  return marker_array;
}


} //namespace manip_helpers

