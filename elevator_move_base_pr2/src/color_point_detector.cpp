#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <algorithm>

class FrameDrawer
{
  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber imgsub_;
  image_transport::Publisher debug_pub_;
  ros::Subscriber pointsub_;
  ros::Publisher result_pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;

  std::string current_frame_;
  double x,y,r;
  CvScalar target_color_;
  double decay_rate_;

  std::vector<tf::StampedTransform> button_pose_;

public:
  FrameDrawer() : private_nh_("~"), it_(private_nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    std::string point_topic = nh_.resolveName("view_target");
    imgsub_ = it_.subscribeCamera(image_topic, 100, &FrameDrawer::imageCb, this);
    pointsub_ = nh_.subscribe(point_topic, 1, &FrameDrawer::pointCb, this);
    debug_pub_ = it_.advertise("debug_image", 1);
    result_pub_ = nh_.advertise<std_msgs::Float32>("light_button", 1);

    int r,g,b;
    private_nh_.param("red", r, 196);
    private_nh_.param("green", g, 196);
    private_nh_.param("blue", b, 64);
    target_color_ = CV_RGB(r,g,b);

    private_nh_.param("decay_rate", decay_rate_, 0.1);
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
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;

    //std::cout << ".";
    // drop 9 of 10 frames
    //if(info_msg->header.seq % 10 != 0) return;
    if(image_msg->header.frame_id != current_frame_) return;

    try {
      // May want to view raw bayer data
      // NB: This is hacky, but should be OK since we have only one image CB.
      if (image_msg->encoding.find("bayer") != std::string::npos)
	boost::const_pointer_cast<sensor_msgs::Image>(image_msg)->encoding = "mono8";

      //image = bridge_.imgMsgToCv(image_msg, "bgr8");
      cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
      image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    // output debug image
    cv::Mat debug_img = image.clone();

    // calcurate the score
    double max_score = -1e9;
    int ix[] = {0,1,0,-1,0}, iy[] = {0,0,1,0,-1};
    for(int ind=0; ind<5; ind++) {
      std::vector<cv::Vec3b> colbuf;
      for(int i = 0; i < 121; i++)
      {
        double px = i/11, py = i%11;
        cv::Point2d uv(x + (5-px)*r/5 + r*ix[ind], y + (5-py)*r/5 + r*iy[ind]);
        if(0<=uv.x && uv.x < image.size().width && 0<=uv.y && uv.y < image.size().height)
        {
          colbuf.push_back(image.at<cv::Vec3b>(static_cast<int>(uv.y), static_cast<int>(uv.x)));
        }
      }

      // check yellow point
      std::vector<double> vscore;
      for(std::vector<cv::Vec3b>::iterator it=colbuf.begin();it!=colbuf.end();it++)
	{
	  double lscore = 0.0;
	  for(int i=0;i<3;i++)
           lscore += exp(-abs(target_color_.val[i] - (*it)[i]) * decay_rate_);
	  vscore.push_back(lscore);
	}

      std::sort(vscore.begin(), vscore.end());
      double score = vscore[vscore.size()*3/4];
      if(max_score < score) { max_score = score; }
    }

    // for debug image
    cv::circle(debug_img, cv::Point(static_cast<int>(x), static_cast<int>(y)), static_cast<int>(r), CV_RGB(255,0,0), 3);
    char text[32];
    snprintf(text, sizeof(text), "brightness = %.3f", max_score);
    cv::putText(debug_img, std::string(text),
                cv::Point(static_cast<int>(x - 100), static_cast<int>(y + 70 + r)),
                0, 0.7, CV_RGB(255,0,0),
                2, 8, false);

    std_msgs::Float32 score_msg;
    score_msg.data = static_cast<float>(max_score);
    result_pub_.publish(score_msg);

    // publish debug image
    cv_bridge::CvImage out_msg;
    out_msg.header = image_msg->header;
    out_msg.encoding = "bgr8";
    out_msg.image = debug_img;
    debug_pub_.publish(out_msg.toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "average_color");
  FrameDrawer drawer;
  ros::spin();
}
