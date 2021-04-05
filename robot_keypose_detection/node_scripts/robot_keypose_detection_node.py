#!/usr/bin/env python
# -*- coding: utf-8 -*-

import chainer
from cameramodels import PinholeCameraModel
from chainer_openpose.links import OpenPoseNet
from chainer_openpose.visualizations import overlay_pose
from chainer_openpose import PoseDetector
import numpy as np
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import wxyz2xyzw
import skrobot
from skrobot.interfaces import PR2ROSRobotInterface

from dynamic_tf_publisher.srv import SetDynamicTF
import geometry_msgs.msg
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport

from robot_keypose_detection.robokey_dataset_utils import pr2_rich_joint_pairs as joint_pairs
from robot_keypose_detection.robokey_dataset_utils import PR2RichJointType as JointType
from robot_keypose_detection.robokey_dataset_utils import pr2_rich_joint_names as pr2_joint_names
from robot_keypose_detection.vision_geometry import solve_pnp


def set_tf(pos, q, parent, child, freq):
    rospy.wait_for_service('/set_dynamic_tf')
    try:
        client = rospy.ServiceProxy(
            '/set_dynamic_tf', SetDynamicTF)
        pose = geometry_msgs.msg.TransformStamped()
        pose.header.frame_id = parent
        pose.child_frame_id = child
        pose.transform.translation = geometry_msgs.msg.Vector3(*pos)
        pose.transform.rotation = geometry_msgs.msg.Quaternion(*q)
        res = client(freq, pose)
        return
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


class RobotKeyPoseDetectorNode(ConnectionBasedTransport):

    def __init__(self):
        super(RobotKeyPoseDetectorNode, self).__init__()

        model = OpenPoseNet(len(JointType) + 1,
                            len(joint_pairs) * 2)

        model_path = rospy.get_param('~model')
        chainer.serializers.load_npz(model_path, model)

        gpu = rospy.get_param('~gpu', 0)
        if gpu >= 0:
            chainer.cuda.get_device_from_id(gpu).use()
            model.to_gpu(gpu)

        self.pose_detector = PoseDetector(
            model, JointType, joint_pairs, device=gpu)

        self.peak_threshold = rospy.get_param('~peak_threshold', 0.1)
        self.target_frame = rospy.get_param('~target_frame', None)

        self.camera_info_msg = None
        self.cm = None
        self.cv_bridge = CvBridge()

        # TODO(whoever) Fix pr2 specific settings.
        robot_model = skrobot.models.PR2()
        self.interface = PR2ROSRobotInterface(robot_model)

        # advertise
        self.pub_camera_info = self.advertise(
            "~output/camera_info", sensor_msgs.msg.CameraInfo,
            queue_size=1)
        self.pub_image = self.advertise("~output/image", sensor_msgs.msg.Image,
                                        queue_size=1)

    def subscribe(self):
        self.sub_camera_info = rospy.Subscriber(
            '~input/camera_info',
            sensor_msgs.msg.CameraInfo,
            queue_size=1,
            callback=self.camera_info_callback)

        self.sub_image = rospy.Subscriber(
            '~input/image',
            sensor_msgs.msg.Image, self.image_cb,
            queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub_image.unregister()

    @property
    def visualize(self):
        return self.pub_image.get_num_connections() > 0

    def camera_info_callback(self, msg):
        self.sub_camera_info.unregister()
        self.camera_info_msg = msg
        self.cm = PinholeCameraModel.from_camera_info(msg)

    def image_cb(self, msg):
        cm = self.cm
        if cm is None:
            rospy.logwarn('Waiting camera info')
            return
        try:
            # transform image to RGB, float, CHW
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = np.asarray(img, dtype=np.float32)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s" % str(e))
            return

        pose_detector = self.pose_detector
        pose, score, all_peaks = pose_detector(img)

        if len(pose) > 0:
            robot_model = self.interface.robot
            robot_model.angle_vector(self.interface.angle_vector())
            poses_3d = []
            poses_2d = []
            for i, joint_name in enumerate(pr2_joint_names):
                # TODO(whoever) Fix pr2 specific settings.
                if joint_name in ['torso_lift_joint',
                                  'wide_stereo_l_stereo_camera_frame_joint',
                                  'wide_stereo_r_stereo_camera_frame_joint']:
                    # skip joint
                    continue
                pose_score = pose[0, i, 3]
                if pose_score < self.peak_threshold:
                    continue
                link = getattr(robot_model, joint_name).child_link
                poses_3d.append(link.worldpos())
                poses_2d.append([pose[0, i, 0], pose[0, i, 1]])
            poses_3d = np.array(poses_3d, 'f')
            poses_2d = np.array(poses_2d, 'f')
            poses_2d = cm.rectify_point(poses_2d)

            if len(poses_3d) >= 4:
                success, quaternion, translation = solve_pnp(
                    poses_3d,
                    poses_2d,
                    cm.K,
                    dist_coeffs=cm.D)
                if success:
                    base_to_cam = Coordinates(
                        pos=translation,
                        rot=quaternion).inverse_transformation()
                    if self.target_frame is None:
                        target_frame = msg.header.frame_id
                    else:
                        target_frame = self.target_frame
                    set_tf(base_to_cam.translation,
                           wxyz2xyzw(base_to_cam.quaternion),
                           'base_footprint',
                           target_frame,
                           30)

        if self.visualize:
            if len(pose) > 0:
                pred_viz = overlay_pose(
                    img, pose[:, :, :3],
                    joint_pairs)
            else:
                pred_viz = img.copy()
            pred_viz = np.array(pred_viz, dtype=np.uint8)
            img_msg = self.cv_bridge.cv2_to_imgmsg(pred_viz, "bgr8")
            img_msg.header = msg.header
            self.pub_image.publish(img_msg)
        self.camera_info_msg.header.stamp = msg.header.stamp
        self.pub_camera_info.publish(self.camera_info_msg)


if __name__ == '__main__':
    rospy.init_node('robot_keypose_detector')
    kpd = RobotKeyPoseDetectorNode()
    rospy.spin()
