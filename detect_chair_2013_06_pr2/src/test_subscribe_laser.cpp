
#include <termios.h>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


#include <pcl17/tracking/tracking.h>
#include <pcl17/tracking/particle_filter.h>
#include <pcl17/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl17/tracking/particle_filter_omp.h>

#include <pcl17/tracking/coherence.h>
#include <pcl17/tracking/distance_coherence.h>
#include <pcl17/tracking/hsv_color_coherence.h>
#include <pcl17/tracking/normal_coherence.h>

#include <pcl17/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl17/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/console/parse.h>
#include <pcl17/common/time.h>
#include <pcl17/common/centroid.h>

#include <pcl17/io/pcd_io.h>

#include <pcl17/filters/passthrough.h>
#include <pcl17/filters/project_inliers.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/approximate_voxel_grid.h>
#include <pcl17/filters/extract_indices.h>

#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/features/integral_image_normal.h>

#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>

#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_polygonal_prism_data.h>
#include <pcl17/segmentation/extract_clusters.h>

#include <pcl17/search/pcl_search.h>
#include <pcl17/common/transforms.h>

#include <boost/format.hpp>
#include <boost/thread.hpp>

using namespace pcl17::tracking;


laser_geometry::LaserProjection* projector_;
tf::TransformListener* tfListener_;
bool save_flag;
int counter;

void
cloud_cb (const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::PointCloud2 point_cloud2;
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr raw_cloud(new pcl17::PointCloud<pcl17::PointXYZ>);

  projector_->transformLaserScanToPointCloud("base_laser_link", *scan, point_cloud2, *tfListener_);
  pcl17::fromROSMsg<pcl17::PointXYZ> (point_cloud2, *raw_cloud);

  ROS_INFO("width : %d height: %d", raw_cloud->width, raw_cloud->height);

  if(save_flag == true){
    save_flag = false;

    pcl17::PCDWriter w;
    std::stringstream ss;
    counter++;

    ss << "laser_scan_snap" << counter << ".pcd";
    w.write<pcl17::PointXYZ>(ss.str(), *raw_cloud, false);
  }

}


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void keyboardCallback()
{
  for(;;)
    {
      try
        {
          int c = getch();
          if(c == 's'){
            save_flag = true;
          }
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
      catch(boost::thread_interrupted&)
        {
          std::cout << "Thread is stopped" << std::endl;
          return;
        }
    }
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "chair_pcl17_tracking");

  ros::NodeHandle n;
  counter = 0;

  tfListener_ = new tf::TransformListener;
  projector_  = new laser_geometry::LaserProjection;
  tfListener_->setExtrapolationLimit(ros::Duration(0.1));

  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 5, &cloud_cb);

  //key board thread
  boost::thread th(&keyboardCallback);

  while(ros::ok()){
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds (100));
  }

  th.interrupt();
  th.join();
  std::cout << "End thread " << std::endl;
}
