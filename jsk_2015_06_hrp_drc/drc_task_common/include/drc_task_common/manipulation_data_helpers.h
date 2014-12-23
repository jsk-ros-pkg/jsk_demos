#include <drc_task_common/InteractiveMarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <jsk_interactive_marker/MarkerDimensions.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/tf.h>

using namespace visualization_msgs;
using namespace std;
namespace manip_helpers
{
//int marker array and objects struct
typedef struct Manipulation_Data{
  //marker (maybe bynaly because the date is too big) 
  drc_task_common::InteractiveMarkerArray int_marker_array;
  //point_cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //box for t-marker
  jsk_interactive_marker::MarkerDimensions dim;
  //pose for t-marker
  geometry_msgs::Pose pose;
}ManipulationData;

YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Pose pose);

void operator >> (const YAML::Node& node, geometry_msgs::Pose& pose);

YAML::Emitter& operator << (YAML::Emitter& out, const jsk_interactive_marker::MarkerDimensions dim);

void operator >> (const YAML::Node& node, jsk_interactive_marker::MarkerDimensions& dim);

bool read_marker(const std::string& file_name, ManipulationData& int_markers);

bool read_marker(int file_index, ManipulationData &imk);

bool write_marker(const std::string& file_name, ManipulationData &imk);
bool write_marker(int file_index, ManipulationData &imk);

std::vector<visualization_msgs::Marker> make_grab(visualization_msgs::InteractiveMarker &int_marker);
visualization_msgs::Marker make_box(float s_x, float s_y, float s_z, float r, float g, float b, float x, float y, float z);
visualization_msgs::Marker make_circle(visualization_msgs::InteractiveMarker &msg );
 visualization_msgs::Marker make_arrow(visualization_msgs::InteractiveMarker &msg, float arrow_offset_x_pos=-0.11, float r=0.8, float g=0.8, float b=0.8);


tf::Transform pose_to_tf(geometry_msgs::Pose pose);

geometry_msgs::Pose tf_to_pose(tf::Transform transform);

geometry_msgs::Pose change_pose(geometry_msgs::Pose base_pose, geometry_msgs::Pose child_pose);

visualization_msgs::MarkerArray make_array(ManipulationData manip_data, double x_pos=0.0, double y_pos=0.0, int id=0, std::string name=std::string("manipulation_visualize_frame"), std::string frame_id_name=std::string("/marker_view"));
}
