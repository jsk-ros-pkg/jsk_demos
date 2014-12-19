#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <drc_task_common/manipulation_data_helpers.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <vector>
using namespace std;
using namespace ros;
using namespace visualization_msgs;
using namespace manip_helpers;

class VisualizeNode
{
public:
  Publisher _manip_visualize_pub;
  ros::NodeHandle _node;
  //std::vector <visualization_msgs::MarkerArray> _marker_array_array;
  visualization_msgs::MarkerArray _marker_array;
  double _x_pos, _y_pos;
  int _x_width;
  double _x_dist_size, _y_dist_size;
  int _id;
  VisualizeNode(){
    //ros::NodeHandle local_nh("~");
    _x_dist_size =0.4; _y_dist_size = 0.4;
    _x_width = 10;
    _x_pos = -_x_width/2 * _x_dist_size; _y_pos = 0; _id = 0;
    
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
	visualization_msgs::MarkerArray marker_array_temp = make_array(*temp_marker, _x_pos, _y_pos, _id);
	_marker_array.markers.insert(_marker_array.markers.end(), marker_array_temp.markers.begin(), marker_array_temp.markers.end());
	_x_pos += _x_dist_size; _id += 1;
	if (_id % _x_width == 0){
	  _x_pos = -_x_width/2 * _x_dist_size; _y_pos+=_y_dist_size;
	}
      }
    }
    //publish
    //_manip_visualize_pub.advertise<>();
    _manip_visualize_pub = _node.advertise<visualization_msgs::MarkerArray>("visualize_manipulation", 1, boost::bind( &VisualizeNode::visualize_connection, this, _1), boost::bind( &VisualizeNode::visualize_disconnection, this, _1));
    
  }
  void visualize_connection(const ros::SingleSubscriberPublisher& pub)
  {
    _manip_visualize_pub.publish(_marker_array);
    ROS_INFO("connection abled size: %td", _marker_array.markers.size());
  }
  void visualize_disconnection(const ros::SingleSubscriberPublisher& pub){    
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_data_visualizer");
  boost::shared_ptr<VisualizeNode> visualize_node(new VisualizeNode());
  ros::spin();
  visualize_node.reset();
  return 0;
}
