#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <posedetection_msgs/ObjectDetection.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>

class FrameDrawer
{
  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber imgsub_;
  image_transport::Publisher pub_;
  ros::Subscriber pointsub_;
  ros::Publisher result_pub_;
  tf::TransformListener tf_listener_;
  sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;

  std::string current_frame_;
  double x,y,r;
  CvScalar target_color_;

  std::vector<tf::StampedTransform> button_pose_;

public:
  FrameDrawer() : it_(nh_), private_nh_("~")
  {
    std::string image_topic = nh_.resolveName("image");
    std::string point_topic = nh_.resolveName("view_target");
    imgsub_ = it_.subscribeCamera(image_topic, 100, &FrameDrawer::imageCb, this);
    pointsub_ = nh_.subscribe(point_topic, 1, &FrameDrawer::pointCb, this);
    pub_ = it_.advertise("image_out", 1);
    result_pub_ = nh_.advertise<std_msgs::Float32>("light_button", 1);

    int r,g,b;
    private_nh_.param("red", r, 196);
    private_nh_.param("green", g, 196);
    private_nh_.param("blue", b, 64);
    target_color_ = CV_RGB(r,g,b);
  }

  void pointCb(const geometry_msgs::PointStamped::ConstPtr& point_msg)
  {
    current_frame_ = point_msg->header.frame_id;
    x = point_msg->point.x;
    y = point_msg->point.y;
    r = point_msg->point.z; // radius on image coordinate
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    IplImage* image = NULL;

    //std::cout << ".";
    // drop 9 of 10 frames
    //if(info_msg->header.seq % 10 != 0) return;
    if(image_msg->header.frame_id != current_frame_) return;

    try {
      // May want to view raw bayer data
      // NB: This is hacky, but should be OK since we have only one image CB.
      if (image_msg->encoding.find("bayer") != std::string::npos)
	boost::const_pointer_cast<sensor_msgs::Image>(image_msg)->encoding = "mono8";

      image = bridge_.imgMsgToCv(image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& ex) {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    std::vector<CvScalar> colbuf;
    for(int i = 0; i < 100; i++){
      double px = i/10, py = i%10;
      cv::Point2d uv(x + (5 - i/10)*r/10, y + (5 - i%10)*r/10);
      if(0<=uv.x && uv.x < image->width-1 && 0<=uv.y && uv.y < image->height-1){
	colbuf.push_back(cvGet2D(image, (int)uv.y, (int)uv.x));
	//cvCircle(image, uv, 1, CV_RGB(255,px*10,py*10), -1);
      }
    }

    // check yello point
    double color_val = 0.0;
    for(std::vector<CvScalar>::iterator it=colbuf.begin();it!=colbuf.end();it++)
    {
      for(int i=0;i<3;i++)
	color_val += exp(-abs(target_color_.val[i] - it->val[i])/10.0);
    }

    double score = color_val / (colbuf.size()+1);
    ROS_INFO("yellow score = %f",score);

    std_msgs::Float32 score_msg;
    score_msg.data = score;
    result_pub_.publish(score_msg);

    pub_.publish(bridge_.cvToImgMsg(image, "bgr8"));
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "average_color");
  FrameDrawer drawer;
  ros::spin();
}
