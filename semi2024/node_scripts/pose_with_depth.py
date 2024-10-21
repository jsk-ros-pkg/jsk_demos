#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division
from __future__ import print_function

from distutils.version import LooseVersion
import pkg_resources
import sys

import cv2
import cv_bridge
import message_filters
import jsk_data
import numpy as np
import os.path as osp
import rospkg
import rospy
from skimage import feature
import torch

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HandPose
from jsk_recognition_msgs.msg import HandPoseArray
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from labanotation.labanotation_utils import coordinates2labanotation
from labanotation.labanotation_utils import cartesian_to_spherical
from labanotation.labanotation_utils import calculate_labanotation


limb_names = ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer',
 'right_eye_inner', 'right_eye', 'right_eye_outer',
 'left_ear', 'right_ear', 'mouth_left', 'mouth_right',
 'left_shoulder', 'right_shoulder', 'left_elbow',
 'right_elbow', 'left_wrist', 'right_wrist',
 'left_pinky', 'right_pinky', 'left_index',
 'right_index', 'left_thumb', 'right_thumb',
 'left_hip', 'right_hip', 'left_knee',
 'right_knee', 'left_ankle', 'right_ankle',
 'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index']



class PoseWithDepth(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pose_pub = self.advertise(
            '~output/pose', HumanSkeletonArray, queue_size=1)
        self.bridge = cv_bridge.CvBridge()

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_skeleton = message_filters.Subscriber(
            # '~input/skeleton'
            '/people_pose_estimation/output/skeleton',
            HumanSkeletonArray,
            queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            # '~input/depth'
            '/camera/depth_registered/hw_registered/image_rect_raw'
            , Image,
            queue_size=1, buff_size=2**24)
        self.subs = [sub_skeleton, sub_depth]

        # NOTE: Camera info is not synchronized by default.
        # See https://github.com/jsk-ros-pkg/jsk_recognition/issues/2165
        sync_cam_info = rospy.get_param("~sync_camera_info", False)
        if sync_cam_info:
            sub_info = message_filters.Subscriber(
                # '~input/info'
                '/camera/rgb/camera_info'
                , CameraInfo, queue_size=1, buff_size=2**24)
            self.subs.append(sub_info)
        else:
            self.sub_info = rospy.Subscriber(
                # '~input/info'
                '/camera/rgb/camera_info'
                , CameraInfo, self._cb_cam_info)

        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        if sync_cam_info:
            sync.registerCallback(self._cb_with_depth_info)
        else:
            self.camera_info_msg = None
            sync.registerCallback(self._cb_with_depth)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()
        if self.sub_info is not None:
            self.sub_info.unregister()
            self.sub_info = None

    def _cb_cam_info(self, msg):
        self.camera_info_msg = msg
        self.sub_info.unregister()
        self.sub_info = None
        rospy.loginfo("Received camera info")

    def _cb_with_depth(self, skeleton_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(skeleton_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(self, skeleton_msg, depth_msg, camera_info_msg):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_msg)
        br = cv_bridge.CvBridge()
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        H, W = depth_img.shape
        for skeleton in skeleton_msg.skeletons:
            limb_to_pose = {}
            for bone_name, bone in zip(skeleton.bone_names,
                                       skeleton.bones):
                u, v = bone.start_point.x, bone.start_point.y
                if 0 <= u < W and 0 <= v < H:
                    z = float(depth_img[int(v)][int(u)])
                else:
                    continue
                if np.isnan(z) or z <= 0:
                    continue
                start_x = (u - camera_model.cx()) * z / camera_model.fx()
                start_y = (v - camera_model.cy()) * z / camera_model.fy()
                start_z = z

                u, v = bone.end_point.x, bone.end_point.y
                if 0 <= u < W and 0 <= v < H:
                    z = float(depth_img[int(v)][int(u)])
                else:
                    continue
                if np.isnan(z) or z <= 0:
                    continue

                a, b = bone_name.split('->')
                limb_to_pose[a] = np.array([bone.start_point.x, bone.start_point.y, bone.start_point.z])
                limb_to_pose[b] = np.array([bone.end_point.x, bone.end_point.y, bone.end_point.z])

                bone.end_point.x = (u - camera_model.cx()) * z / camera_model.fx()
                bone.end_point.y = (v - camera_model.cy()) * z / camera_model.fy()
                bone.end_point.z = z

                bone.start_point.x = start_x
                bone.start_point.y = start_y
                bone.start_point.z = start_z

            if 'left_hip' in limb_to_pose and 'right_hip' in limb_to_pose \
               and 'left_shoulder' in limb_to_pose and 'right_shoulder' in limb_to_pose \
               and 'left_elbow' in limb_to_pose and 'right_elbow' in limb_to_pose \
               and 'left_wrist' in limb_to_pose and 'right_wrist' in limb_to_pose:
                spine_position = (limb_to_pose['left_hip'] + limb_to_pose['right_hip']) / 2.0
                _, _, _, _, elbow_r, elbow_l, wrist_r, wrist_l = calculate_labanotation(spine_position,
                                       limb_to_pose['right_shoulder'],
                                       limb_to_pose['left_shoulder'],
                                       limb_to_pose['right_elbow'],
                                       limb_to_pose['left_elbow'],
                                       limb_to_pose['right_wrist'],
                                       limb_to_pose['left_wrist'])
                print(elbow_r, wrist_r, elbow_l, wrist_l)



        self.pose_pub.publish(skeleton_msg)


if __name__ == '__main__':
    rospy.init_node('pose_with_depth')
    PoseWithDepth()
    rospy.spin()
