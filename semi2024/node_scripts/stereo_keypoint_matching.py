#!/usr/bin/env python3

import os

from scipy import linalg
import numpy as np

from jsk_topic_tools import ConnectionBasedTransport

# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
    from cameramodels import PinholeCameraModel
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    from cameramodels import PinholeCameraModel
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

import rospy
from sensor_msgs.msg import CameraInfo
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import PeoplePose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment
import message_filters


try:
    from sklearn.utils.linear_assignment_ import linear_assignment as _linear_assignment
    def linear_assignment(dists):
        xy = _linear_assignment(dists)
        return xy[:, 0], xy[:, 1]
except:
    from scipy.optimize import linear_sum_assignment as linear_assignment

from warnings import filterwarnings
filterwarnings(action='ignore', category=DeprecationWarning,
               message='`np.bool` is a deprecated alias for the builtin `bool`. To silence this warning, use `bool` by itself. Doing this will not modify any behavior and is safe. If you specifically wanted the numpy scalar type, use `np.bool_` here.')



def DLT(P1, P2, point1, point2):
    A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:],]
    A = np.array(A).reshape((4,4))
    B = np.dot(A.transpose(), A)
    U, s, Vh = linalg.svd(B, full_matrices=False)
    return Vh[3, 0:3] / Vh[3, 3]


class StereoKeypointMatching(ConnectionBasedTransport):

    def __init__(self):
        super(StereoKeypointMatching, self).__init__()

        self.pose_pub = self.advertise(
            '~output/pose', PeoplePoseArray, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 30)
        sub_left_camera_info = message_filters.Subscriber(
            '~left/camera_info',
            CameraInfo, queue_size=1)
        sub_right_camera_info = message_filters.Subscriber(
            '~right/camera_info',
            CameraInfo, queue_size=1)
        sub_right_skeleton = message_filters.Subscriber(
            '~right/skeleton',
            HumanSkeletonArray, queue_size=1)
        sub_left_skeleton = message_filters.Subscriber(
            '~left/skeleton',
            HumanSkeletonArray, queue_size=1)
        self.subs = [
            sub_left_camera_info,
            sub_right_camera_info,
            sub_left_skeleton,
            sub_right_skeleton,
        ]
        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def poses_from_skeletons(self, skeleton_msg):
        limbs = []
        poses = []
        connections = []
        for skeleton in skeleton_msg.skeletons:
            limb_to_pose = {}
            for bone_name, bone in zip(skeleton.bone_names,
                                       skeleton.bones):
                a, b = bone_name.split('->')
                connections.append((a, b))
                limbs.extend([a, b])
                limb_to_pose[a] = np.array(
                    [bone.start_point.x,
                     bone.start_point.y,
                     bone.start_point.z])
                limb_to_pose[b] = np.array(
                    [bone.end_point.x,
                     bone.end_point.y,
                     bone.end_point.z])
            poses.append(limb_to_pose)
        return poses, limbs, connections

    def callback(self, left_camera_info, right_camera_info,
                 left_hand, right_hand):
        left_camera = PinholeCameraModel.from_camera_info(
            left_camera_info)
        right_camera = PinholeCameraModel.from_camera_info(
            right_camera_info)

        left_poses, left_limbs, c1 = self.poses_from_skeletons(left_hand)
        right_poses, right_limbs, c2 = self.poses_from_skeletons(right_hand)
        connections = list(set(c1 + c2))
        limbs = list(set(left_limbs + right_limbs))

        dists = np.zeros((len(left_hand.skeletons), len(right_hand.skeletons)),
                         dtype=np.float64)
        for i, left_pose in enumerate(left_poses):
            for j, right_pose in enumerate(right_poses):
                dist = 0
                for name in limbs:
                    if not (name in left_pose and name in right_pose):
                        continue
                    left_xy = left_pose[name]
                    right_xy = right_pose[name]
                    point = DLT(left_camera.P, right_camera.P, left_xy, right_xy)
                    x, y = left_camera.project3d_to_pixel(point)
                    dist += np.sqrt((left_xy[0] - x) ** 2 + (left_xy[1] - y) ** 2)
                    x, y = right_camera.project3d_to_pixel(point)
                    dist += np.sqrt((right_xy[0] - x) ** 2 + (right_xy[1] - y) ** 2)
                dists[i, j] = dist

        left_indices, right_indices = linear_assignment(dists)

        out_msg = PeoplePoseArray(header=left_camera_info.header)
        hand_list = []
        for left_index, right_index in zip(left_indices, right_indices):
            left_pose = left_poses[left_index]
            right_pose = right_poses[right_index]
            pose_msg = PeoplePose()
            hand = {}
            for name in limbs:
                if not (name in left_pose and name in right_pose):
                    continue
                left_xy = left_pose[name]
                right_xy = right_pose[name]
                x, y, z = DLT(left_camera.P, right_camera.P, left_xy, right_xy)
                u, v = right_camera.project3d_to_pixel((x, y, z))
                lu, lv = left_camera.project3d_to_pixel((x, y, z))
                if not (0 <= u < right_camera.width and 0 <= v < right_camera.height):
                    continue
                if not (0 <= lu < left_camera.width and 0 <= lv < left_camera.height):
                    continue
                # diff = np.sqrt((right_xy[0] - u) ** 2 + (right_xy[1] - v) ** 2)
                # if diff >= 5:
                #     continue
                # diff = np.sqrt((left_xy[0] - lu) ** 2 + (left_xy[1] - lv) ** 2)
                # if diff >= 5:
                #     continue
                pose_msg.limb_names.append(name)
                pose_msg.poses.append(
                    Pose(position=Point(x=x, y=y, z=z),
                         orientation=Quaternion(w=1.0)))
                hand[name] = np.array([x, y, z])
            hand_list.append(hand)
            out_msg.poses.append(pose_msg)
        self.pose_pub.publish(out_msg)

        if self.skeleton_pub.get_num_connections() > 0:
            skeleton_msgs = HumanSkeletonArray(header=left_camera_info.header)
            for hand in hand_list:
                skeleton_msg = HumanSkeleton(header=left_camera_info.header)
                for a, b in connections:
                    if not (a in hand and b in hand):
                        continue
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*hand[a]),
                        end_point=Point(*hand[b]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)
                skeleton_msgs.skeletons.append(skeleton_msg)
            self.skeleton_pub.publish(skeleton_msgs)


if __name__ == '__main__':
    rospy.init_node('stereo_keypoint_matching')
    node = StereoKeypointMatching()
    rospy.spin()
