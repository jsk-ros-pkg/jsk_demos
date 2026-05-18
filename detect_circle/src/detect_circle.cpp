// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
#include <ros/ros.h>
#include <opencv_apps/CircleArrayStamped.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
// #include <detect_circle/PointStampedArray.h>
#include <iostream>



// // initial and max values of the parameters of interests.
// const int canny_threshold_initial_value = 100;
// const int accumulator_threshold_initial_value = 50;
// // const int max_accumulator_threshold = 200;
// // const int max_canny_threshold = 255;

// int canny_threshold = canny_threshold_initial_value;
// int accumulator_threshold = accumulator_threshold_initial_value;

// class DetectCircle
// {
//     private:
//         ros::NodeHandle nh;
//         // ros::NodeHandle pnh("~");
//         ros::Subscriber sub;
//         ros::Publisher pub_img;
//         ros::Publisher pub_points;
        
//     public:
//         DetectCircle()
//         {
//             sub = nh.subscribe("/usb_cam/image_raw", 1, &DetectCircle::callback, this);
//             pub_img = nh.advertise<sensor_msgs::Image>("/camera/image/circle", 1);
//             pub_points = nh.advertise<detect_circle::PointStampedArray>("/camera/points/circle", 1);
//             pnh.getParam("canny_threshold", canny_threshold);
//             pnh.getParam("accumulator_threshold", accumulator_threshold);
//         }

//         ~DetectCircle(){};

//         void callback(const sensor_msgs::ImageConstPtr& msg)
//         {
//             cv_bridge::CvImagePtr cv_ptr;
//             try
//             {
//                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             }
//             catch(cv_bridge::Exception& e)
//             {
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//                 return;
//             }

//             cv::Mat src, src_gray;

//             src = cv_ptr->image;

//             if(src.empty())
//             {
//                 std::cerr << "Invalid input image\n";
//                 return;
//             }

//             // Convert it to gray
//             cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);

//             // Reduce the noise so we avoid false circle detection
//             cv::GaussianBlur(src_gray, src_gray, cv::Size(9, 9), 2, 2);

//             detect_circle::PointStampedArray points;
//             points.header = msg->header;

//             // will hold the results of the detection
//             std::vector<cv::Vec3f> circles;
//             // runs the actual detection
//             cv::HoughCircles(src_gray, circles, cv::HOUGH_GRADIENT, 1, src_gray.rows/8, canny_threshold, accumulator_threshold, 0, 0);

//             // clone the colour, input image for displaying purposes
//             cv::Mat img = src.clone();
//             points.points.resize(circles.size());
//             for(size_t i = 0; i < circles.size(); i++)
//             {
//                 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//                 int radius = cvRound(circles[i][2]);
//                 points.points[i].x = center.x;
//                 points.points[i].y = center.y;
//                 points.points[i].z = 0;
//                 // circle center
//                 cv::circle(img, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
//                 // circle outline
//                 cv::circle(img, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
//             }

//             sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

//             ROS_INFO("Number of circles detected: %ld", circles.size());

//             for (size_t i = 0; i < circles.size(); i++)
//             {
//                 ROS_INFO("Circle #%ld: center at (%d, %d)", i, (int)(points.points[i].x), (int)(points.points[i].y));
//             }

//             pub_img.publish(img_msg);
//             pub_points.publish(points);     
//         }

// };

class ConvertHoughCircle2PointStamped
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        
    public:
        ConvertHoughCircle2PointStamped()
        {
            sub = nh.subscribe("/hough_circles/circles", 1, &ConvertHoughCircle2PointStamped::callback, this);
            pub = nh.advertise<geometry_msgs::PointStamped>("/camera/circle/point", 1);
        }

        ~ConvertHoughCircle2PointStamped(){};

        void callback(const opencv_apps::CircleArrayStamped& msg)
        {
            geometry_msgs::PointStamped point;
            if (msg.circles.size() == 0)
            {
                ROS_INFO("No circle detected");
                return;
            }
            point.header = msg.header;
            point.point.x = msg.circles[0].center.x;
            point.point.y = msg.circles[0].center.y;
            point.point.z = 0;

            ROS_INFO("Circle center at (%f, %f)", point.point.x, point.point.y);

            pub.publish(point);
        }

};

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "detect_circle");
    // DetectCircle dc;
    // ROS_INFO("detect_circle node has started");
    // ros::spin();

    ros::init(argc, argv, "detect_circle");
    ConvertHoughCircle2PointStamped chc2ps;
    ROS_INFO("convert_hough_circle2point_stamped node has started");
    ros::spin();

    return 0;
}
