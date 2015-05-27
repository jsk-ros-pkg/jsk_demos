#!/usr/bin/env python

import rospy
import sys
import os
import math


from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage,
                                  checkTopicIsPublished)

from sensor_msgs.msg import PointCloud2, Image
from pcl_msgs.msg import PointIndices

if __name__ == "__main__":
    rospy.init_node("chesk_sanity_fc")
    
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

