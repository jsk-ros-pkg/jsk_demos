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


# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
    import mediapipe.python.solutions.drawing_styles
    import mediapipe as mp
    ds = mp.solutions.drawing_styles
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    import mediapipe.python.solutions.drawing_styles
    import mediapipe as mp
    ds = mp.solutions.drawing_styles
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


class PeoplePoseEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(PeoplePoseEstimation, self).__init__()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose

        self.connections = mp.solutions.pose.POSE_CONNECTIONS
        self.names = [i.name.lower()
                      for i in mp.solutions.pose.PoseLandmark]
        print(self.names)

        self.people_pose_estimator = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        self.bridge = CvBridge()
        self.pub_img = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pose_pub = self.advertise('~output/pose',
                                       PeoplePoseArray, queue_size=1)
        self.pub_img_compressed = self.advertise(
            '~output/viz/compressed',
            CompressedImage, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)

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
        mp_pose = self.mp_pose

        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.people_pose_estimator.process(image)

        people_pose_msg = PeoplePoseArray(header=img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)
        _PRESENCE_THRESHOLD = 0.5
        _VISIBILITY_THRESHOLD = 0.5
        if results.pose_landmarks:
            pose_list = []
            for pose_landmarks in [results.pose_landmarks]:
                pose_msg = PeoplePose()
                image_rows, image_cols, _ = image.shape
                pose = {}
                for idx, landmark in enumerate(pose_landmarks.landmark):
                    if ((landmark.HasField('visibility') and
                         landmark.visibility < _VISIBILITY_THRESHOLD) or
                        (landmark.HasField('presence') and
                         landmark.presence < _PRESENCE_THRESHOLD)):
                        continue
                    landmark_px = self.mp_drawing._normalized_to_pixel_coordinates(
                        landmark.x, landmark.y,
                        image_cols, image_rows)
                    if landmark_px:
                        pose_msg.scores.append(landmark.visibility)
                        pose_msg.limb_names.append(self.names[idx])
                        pose_msg.poses.append(
                            Pose(position=Point(x=landmark_px[0],
                                                y=landmark_px[1],
                                                z=0)))
                        pose[self.names[idx]] = np.array([landmark_px[0],
                                                          landmark_px[1],
                                                          0.0])
                people_pose_msg.poses.append(pose_msg)
                pose_list.append(pose)

            for pose in pose_list:
                skeleton_msg = HumanSkeleton(header=img_msg.header)
                for a, b in self.connections:
                    a = self.names[a]
                    b = self.names[b]
                    if not (a in pose and b in pose):
                        continue
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*pose[a]),
                        end_point=Point(*pose[b]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)
                skeleton_msgs.skeletons.append(skeleton_msg)
        self.pose_pub.publish(people_pose_msg)
        self.skeleton_pub.publish(skeleton_msgs)


        if self.pub_img.get_num_connections() > 0 or self.pub_img_compressed.get_num_connections() > 0:
            # Draw the pose annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.pose_landmarks:
                for pose_landmarks in [results.pose_landmarks]:
                    mp_drawing.draw_landmarks(
                        image, pose_landmarks, self.connections,)
                        # landmark_drawing_spec=mp.solutions.drawing_styles.get_default_pose_landmarks_style())

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
    rospy.init_node('people_pose_estimation')
    node = PeoplePoseEstimation()
    rospy.spin()
