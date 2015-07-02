#!/usr/bin/env python

import rospy
import sys
import os
import math
import re
from colorama import Fore, Style
from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage, indexMessage,
                                  checkTopicIsPublished,
                                  checkROSMasterCLOSE_WAIT, checkNodeState,
                                  checkSilverHammerSubscribe,
                                  checkGitRepoWithRosPack,
                                  checkBlackListDaemon)
from std_msgs.msg import Time
from sensor_msgs.msg import PointCloud2, Image
from jsk_recognition_msgs.msg import (ModelCoefficientsArray,
                                      PolygonArray,
                                      ClusterPointIndices,
                                      BoundingBoxArray,
                                      Torus)
from pcl_msgs.msg import PointIndices
from geometry_msgs.msg import PolygonStamped, Point32
from jsk_hrp2_ros_bridge.sanity_util import checkMultisenseRemote

if __name__ == "__main__":
    indexMessage("Check Daemons in FC")
    checkBlackListDaemon(["chrome", "dropbox", "skype"], kill=False)
    
    host = re.match("http://([0-9a-zA-Z]*):.*", os.environ["ROS_MASTER_URI"]).groups(0)[0]
    checkROSMasterCLOSE_WAIT(host)

    indexMessage("Check Git Repos in FC")
    checkGitRepoWithRosPack("drc_task_common")
    rospy.init_node("chesk_sanity_fc")

    indexMessage("Check Nodes in FC")
    checkNodeState("/fc_to_ocs_basic_low_speed", True)
    checkNodeState("/fc_to_ocs_eus", True)
    checkNodeState("/fc_to_ocs_low_speed", True)
    checkNodeState("/fc_to_ocs_vehicle", True)
    checkNodeState("/fc_from_ocs_eus", True)
    checkNodeState("/fc_from_ocs_low_speed", True)
    checkNodeState("/fc_from_ocs_vehicle", True)
    checkNodeState("/fc_from_ocs_reconfigure", True)
    checkNodeState("/highspeed_streamer", True)

    indexMessage("Check Topics in FC")
    checkMultisenseRemote(
        "multisense remote is not working."
        "Run following command:\n"
        "roslaunch jaxon_ros_bridge jaxon_multisense_remote.launch\n"
        "on your machine")
    checkTopicIsPublished(
        "/highspeed_streamer/last_send_time",
        Time,
        "Silverhammer highspeed protocol is working",
        "Silverhammer highspeed protocol is not working")
    checkTopicIsPublished(
        "/fc_from_ocs_low_speed/last_received_time",
        Time,
        "Silverhammer lowspeed protocol is working",
        "Silverhammer lowspeed protocol is not working",
        5,
        [["/fc_from_ocs_reconfigure/last_received_time", Time],
         ["/fc_from_ocs_vehicle/last_received_time", Time],
         ["/fc_from_ocs_eus/last_received_time", Time],
         ["/fc_to_ocs_eus/last_send_time", Time],
         ["/fc_to_ocs_low_speed/last_send_time", Time],
         ["/fc_to_ocs_basic_low_speed/last_send_time", Time]])
    checkTopicIsPublished(
        "/communication/point_cloud",
        PointCloud2,
        "Silverhammer highspeed input is available",
        "Silverhammer highspeed input is not working",
        5,
        [["/communication/laser_cloud", PointCloud2],
         ["/communication/image_rect_color", Image],
         ["/communication/panorama_image", Image]])
    checkTopicIsPublished(
        "/laser_preprocess/tilt_laser_listener/output_cloud",
        PointCloud2,
        "laser preprocess is ok",
        "laser_preprocess do not working",
        5,
        [["/laser_preprocess/downsampler/output", PointCloud2],
         ["/laser_preprocess/gsensor_cloud/output", PointCloud2],
         ["/laser_preprocess/camera_laser_cloud/output", PointCloud2],
         ["/laser_preprocess/x_filter/output", PointCloud2],
         ["/laser_preprocess/y_filter/output", PointCloud2],
         ["/laser_preprocess/z_filter/output", PointCloud2],
         ["/laser_preprocess/odom_cloud/output", PointCloud2]]
    )
    checkTopicIsPublished(
        "/stereo_preprocessing/normal_estimation/output",
        PointCloud2,
        "stereo preprocessing is working",
        "stereo preprocessing is not working",
        5,
        [["/stereo_preprocessing/stereo_ground_cloud/output",
          PointCloud2],
         ["/stereo_preprocessing/stereo_downsampled_ground_cloud/output",
          PointCloud2]])
    checkTopicIsPublished(
        "/door_recognition/mask_image_generator/output", Image,
        "door recognition is working (not fully checked)",
        "doro recognition is not working",
        5,
        [["/door_recognition/camera_mask_image_filter/output", PointIndices],
         ["/door_recognition/camera_filtered_cloud/output", PointCloud2]])
    checkTopicIsPublished(
        "/drill_recognition/x_filter/output", PointCloud2,
        "drill recognition is working",
        "drill recognition is not working",
        5,
        [["/drill_recognition/y_filter/output", PointCloud2],
         ["/drill_recognition/z_filter/output", PointCloud2],
         ["/drill_recognition/normal_estimation/output", PointCloud2],
         ["/drill_recognition/normal_estimation/output_with_xyz", PointCloud2],
         ["/drill_recognition/region_growing_multiple_plane_segmentation/output/coefficients", ModelCoefficientsArray],
         ["/drill_recognition/polygon_flipper/output/polygons", PolygonArray],
         ["/drill_recognition/multi_plane_extraction/output", PointCloud2],
         ["/drill_recognition/normal_direction_filter/output", PointIndices],
         ["/drill_recognition/cluster_point_indices_decomposer/boxes",
          BoundingBoxArray],
         ["/drill_recognition/filtered_points/output", PointCloud2],
         ["/drill_recognition/region_growing_multiple_plane_segmentation/output/inliers", ClusterPointIndices],
         ["/drill_recognition/filter_by_position/output",
          BoundingBoxArray],
         ["/drill_recognition/remove_small_noises/output",
          PointCloud2],
         ["/drill_recognition/polygon_flipper/output/coefficients",
          ModelCoefficientsArray],
         ["/drill_recognition/euclidean_clustering/output",
          ClusterPointIndices],
         ["/drill_recognition/region_growing_multiple_plane_segmentation/output/polygons",
          PolygonArray]])
    checkTopicIsPublished(
        "/drill_recognition_for_wall/downsampler/output", PointCloud2,
        "drill recognition for wall is working (not fully checked)",
        "drill recognition for wall is not working",
        5,
        [["/drill_recognition_for_wall/mask_image_generator/output",
          Image],
         ["/drill_recognition_for_wall/mask_image_filter/output",
          PointIndices],
         ["/drill_recognition_for_wall/organized_points_converter/output_cloud",
          PointCloud2],
         ["/drill_recognition_for_wall/normal_estimation/output",
          PointCloud2],
         ["/drill_recognition_for_wall/normal_estimation/output_with_xyz",
          PointCloud2],
         ["/drill_recognition_for_wall/plane_segmentation/output/coefficients",
          ModelCoefficientsArray],
         ["/drill_recognition_for_wall/plane_segmentation/output/polygons",
          PolygonArray]
     ])
    checkTopicIsPublished("/footcoords/state", None, echo=True)

    indexMessage("Check SilverHammer Subscribe Hz in FC")
    checkSilverHammerSubscribe("/fc_to_ocs_basic_low_speed/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/fc_to_ocs_eus/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/fc_to_ocs_low_speed/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/fc_to_ocs_vehicle/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/highspeed_streamer/last_send_time", 1.0, 0.4, timeout=7)

    checkSilverHammerSubscribe("/fc_from_ocs_eus/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/fc_from_ocs_low_speed/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/fc_from_ocs_reconfigure/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/fc_from_ocs_vehicle/last_received_time", 10.0, 8.0, timeout=7)

    indexMessage("Check  Valve Recognition in FC")
    # Publish dummy input for valve recognition
    pub = rospy.Publisher("/valve_recognition/input_rect", PolygonStamped, latch = True)
    dummy_polygon = PolygonStamped()
    dummy_polygon.header.frame_id = "left_camera_optical_frame"
    dummy_polygon.header.stamp = rospy.Time.now()
    dummy_polygon.polygon.points.append(Point32(x = 0, y = 0))
    # dummy_polygon.polygon.points.append(Point32(x = 100, y = 0))
    dummy_polygon.polygon.points.append(Point32(x = 100, y = 100))
    # dummy_polygon.polygon.points.append(Point32(x = 0, y = 100))
    pub.publish(dummy_polygon)
    checkTopicIsPublished(
        "/valve_recognition/rect_to_mask_image/output", Image,
        "valve recognition is working (not fully checked)",
        "valve recognition is not working (not fully checked)",
        5,
        [["/valve_recognition/mask_image_filter/output", PointIndices],
         ["/valve_recognition/filtered_cloud/output", PointCloud2],
         ["/valve_recognition/normal_estimation/output", PointCloud2],
         ["/valve_recognition/normal_estimation/output_with_xyz", PointCloud2],
         ["/valve_recognition/normal_direction_filter/output", PointIndices],
         ["/valve_recognition/euclidean_clustering/output", ClusterPointIndices],
         ["/valve_recognition/cluster_decomposer/boxes", BoundingBoxArray]# ,
         # ["/valve_recognition/nearest_cluster/output/indices", PointIndices],
         # ["/valve_recognition/valve_finder/output", Torus]
     ])

    indexMessage("Check Car Recognition in FC")
    checkNodeState("/drive/recognition/car_path_visualizer", True)
    checkNodeState("/drive/recognition/marker_dynamic_tf_publisher", True)
    checkNodeState("/drive/recognition/jaxon_TF_BODY_marker", True, sub_fail="*** IF YOU USE HRP2, DON'T WORRY ABOUNT THIS ***")
    checkNodeState("/drive/recognition/jaxon_TF_marker_car_center", True, sub_fail="*** IF YOU USE HRP2, DON'T WORRY ABOUNT THIS ***")
    checkNodeState("/drive/recognition/hrp2_TF_BODY_marker", True, sub_fail="*** IF YOU USE JAXON, DON'T WORRY ABOUNT THIS ***")
    checkNodeState("/drive/recognition/hrp2_TF_marker_car_center", True, sub_fail="*** IF YOU USE JAXON, DON'T WORRY ABOUNT THIS ***")

    checkTopicIsPublished("/communication/drive/recognition/predicted_path/marker", None, "drive recognition is working", "drive recognition is not working")
