#!/usr/bin/env python3

import os
import sys

from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment

opencv_apps_enabled = True
try:
    from opencv_apps.msg import FaceArrayStamped
    from opencv_apps.msg import Face
    from opencv_apps.msg import Rect
except ImportError:
    opencv_apps_enabled = False
opencv_apps_enabled = False


# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
    import mediapipe as mp
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    import mediapipe as mp
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    from cv_bridge import CvBridge
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(ws_python3_paths[0])
    else:
        from cv_bridge import CvBridge


class FaceRecognition(ConnectionBasedTransport):

    def __init__(self):
        super(FaceRecognition, self).__init__()

        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils

        self.connections = mp.solutions.pose.POSE_CONNECTIONS
        self.names = [i.name.lower()
                      for i in mp.solutions.pose.PoseLandmark]

        self.connections = [
            (self.mp_face_detection.FaceKeyPoint.RIGHT_EYE, self.mp_face_detection.FaceKeyPoint.RIGHT_EAR_TRAGION),
            (self.mp_face_detection.FaceKeyPoint.NOSE_TIP, self.mp_face_detection.FaceKeyPoint.RIGHT_EYE),
            (self.mp_face_detection.FaceKeyPoint.LEFT_EYE, self.mp_face_detection.FaceKeyPoint.LEFT_EAR_TRAGION),
            (self.mp_face_detection.FaceKeyPoint.NOSE_TIP, self.mp_face_detection.FaceKeyPoint.LEFT_EYE),
            (self.mp_face_detection.FaceKeyPoint.MOUTH_CENTER, self.mp_face_detection.FaceKeyPoint.NOSE_TIP),
        ]
        self.names = [i.name.lower()
                      for i in self.mp_face_detection.FaceKeyPoint]

        self.face_estimator = self.mp_face_detection.FaceDetection(
            min_detection_confidence=0.5)

        self.bridge = CvBridge()
        self.pub_img = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pub_img_compressed = self.advertise(
            '~output/viz/compressed',
            CompressedImage, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)
        if opencv_apps_enabled:
            self.faces_pub = self.advertise(
                '~faces',
                FaceArrayStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        bridge = self.bridge

        mp_drawing = self.mp_drawing
        mp_face_detection = self.mp_face_detection

        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.face_estimator.process(image)
        image_rows, image_cols, _ = image.shape

        if opencv_apps_enabled:
            face_msgs = FaceArrayStamped(header=img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)
        if results.detections:
            for detection in results.detections:
                location = detection.location_data
                relative_bounding_box = location.relative_bounding_box
                xy = self.mp_drawing._normalized_to_pixel_coordinates(
                    relative_bounding_box.xmin, relative_bounding_box.ymin,
                    image_cols, image_rows)
                if xy is None:
                    continue
                x1, y1 = xy
                xy = self.mp_drawing._normalized_to_pixel_coordinates(
                    relative_bounding_box.xmin + relative_bounding_box.width,
                    relative_bounding_box.ymin + relative_bounding_box.height,
                    image_cols, image_rows)
                if xy is None:
                    continue
                x2, y2 = xy
                if opencv_apps_enabled:
                    face_msg = Face()
                    face_msg.face = Rect(x=x1, y=y1,
                                         width=x2 - x1, height=y2 - y1)
                    face_msgs.faces.append(face_msg)

                skeleton_msg = HumanSkeleton(header=img_msg.header)
                index2pixel = {}
                for i, keypoint in enumerate(location.relative_keypoints):
                    if keypoint is None:
                        continue
                    keypoint_px = self.mp_drawing._normalized_to_pixel_coordinates(
                        keypoint.x, keypoint.y,
                        image_cols, image_rows)
                    index2pixel[i] = [keypoint_px[0], keypoint_px[1], 0.0]
                for a_index, b_index in self.connections:
                    if not (a_index in index2pixel and b_index in index2pixel):
                        continue
                    a = self.names[a_index]
                    b = self.names[b_index]
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*index2pixel[a_index]),
                        end_point=Point(*index2pixel[b_index]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)

                skeleton_msgs.skeletons.append(skeleton_msg)
        if opencv_apps_enabled:
            self.faces_pub.publish(face_msgs)
        self.skeleton_pub.publish(skeleton_msgs)

        if self.pub_img.get_num_connections() > 0 or self.pub_img_compressed.get_num_connections() > 0:
            # Draw the pose annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.detections:
                for detection in results.detections:
                    mp_drawing.draw_detection(image, detection)

        if self.pub_img.get_num_connections() > 0:
            out_img_msg = bridge.cv2_to_imgmsg(
                image, encoding='bgr8')
            out_img_msg.header = img_msg.header
            self.pub_img.publish(out_img_msg)

        if self.pub_img_compressed.get_num_connections() > 0:
            # publish compressed http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber  # NOQA
            vis_compressed_msg = CompressedImage()
            vis_compressed_msg.header = img_msg.header
            # image format https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_image_transport/src/compressed_publisher.cpp#L116  # NOQA
            vis_compressed_msg.format = 'bgr8' + '; jpeg compressed bgr8'
            vis_compressed_msg.data = np.array(
                cv2.imencode('.jpg', image)[1]).tobytes()
            self.pub_img_compressed.publish(vis_compressed_msg)


if __name__ == '__main__':
    rospy.init_node('face_recognition')
    node = FaceRecognition()
    rospy.spin()
