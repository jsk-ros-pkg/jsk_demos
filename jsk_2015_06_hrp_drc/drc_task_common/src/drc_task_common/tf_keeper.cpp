#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

typedef struct TF_STRUCTURE {
  tf::Transform transform;
  std::string child_name;
  std::string parent_name;
  TF_STRUCTURE(geometry_msgs::TransformStamped transform_msg){
    transform = tf::Transform(tf::Quaternion(transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w), tf::Vector3(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z));
    child_name = transform_msg.child_frame_id;
    parent_name = transform_msg.header.frame_id;
  }
} tf_struct;


class TfKeeper
{
  ros::NodeHandle _node;
  ros::Subscriber _sub_single;
  ros::Subscriber _sub_array;
  ros::Timer _tf_timer;
  tf::TransformBroadcaster _br;
  std::vector<tf_struct> _transforms;
public:
  TfKeeper():_tf_timer(_node.createTimer(ros::Duration(0.02), &TfKeeper::timerCallback, this)){
    _sub_single = _node.subscribe("transform_single", 1, &TfKeeper::setTransform, this);
    _sub_array = _node.subscribe("transform_array", 1, &TfKeeper::setTransformArray, this);
  }
  ~TfKeeper(){
  }
  void timerCallback(const ros::TimerEvent&){
    for (std::vector<tf_struct>::iterator it = _transforms.begin(); it != _transforms.end(); it++){ 
      _br.sendTransform(tf::StampedTransform(it->transform, ros::Time::now(), it->parent_name , it->child_name));
    }
  }
  void setTransform(geometry_msgs::TransformStamped transform_msg){
    for(std::vector<tf_struct>::iterator it = _transforms.begin(); it != _transforms.end(); it++){
      if(it->child_name == transform_msg.child_frame_id){
	*it = tf_struct(transform_msg);
	return;
      }
    }
    _transforms.push_back(tf_struct(transform_msg));
  }
  void setTransformSingle(const geometry_msgs::TransformStampedConstPtr &transform_msg_single){
    setTransform(*transform_msg_single);
  }
  void setTransformArray(const tf2_msgs::TFMessageConstPtr &transform_msg_array){
    for(size_t i=0; i<transform_msg_array->transforms.size(); i++){
      setTransform(transform_msg_array->transforms[i]);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_keeper");
  boost::shared_ptr<TfKeeper> tf_keeper(new TfKeeper());
  ros::spin();
  tf_keeper.reset();
  return 0;
}
