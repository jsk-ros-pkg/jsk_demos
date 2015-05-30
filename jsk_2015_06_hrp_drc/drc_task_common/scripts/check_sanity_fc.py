#!/usr/bin/env python

import rospy
import sys
import os
import math


from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage,
                                  checkTopicIsPublished)
from std_msgs.msg import Time
from sensor_msgs.msg import PointCloud2, Image
from pcl_msgs.msg import PointIndices
from jsk_hrp2_ros_bridge.sanity_util import checkMultisenseRemote

if __name__ == "__main__":
    rospy.init_node("chesk_sanity_fc")
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
        "Silverhammer highspeed is working",
        "Silverhammer highspeed is not working",
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
        [["/stereo_preprocessing/stereo_ground_cloud/output", PointCloud2],
         ["/stereo_preprocessing/stereo_downsampled_ground_cloud/output", PointCloud2],
         ["/stereo_preprocessing/stereo_downsampled_ground_cloud/output", PointCloud2]])
    checkTopicIsPublished(
        "/door_recognition/mask_image_generator/output", Image,
        "door recognition is working (not fully checked)",
        "doro recognition is not working",
        5,
        [["/door_recognition/camera_mask_image_filter/output", PointIndices],
         ["/door_recognition/camera_filtered_cloud/output", PointCloud2]])

