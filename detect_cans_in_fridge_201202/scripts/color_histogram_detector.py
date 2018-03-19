#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import message_filters
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Quaternion
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from posedetection_msgs.msg import ObjectDetection, Object6DPose
import numpy as np
import tf.transformations as T


class ColorHistogramDetector(ConnectionBasedTransport):
    def __init__(self):
        super(ColorHistogramDetector, self).__init__()

        self.queue_size = rospy.get_param("~queue_size", 100)

        self.publish_tf = rospy.get_param("~publish_tf", True)
        if self.publish_tf:
            self.tfb = tf2_ros.TransformBroadcaster()

        self.align_box_pose = rospy.get_param("~align_box_pose", True)
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", None)
        if self.fixed_frame_id is not None:
            self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
            if not self.tfl.wait_for_server(rospy.Duration(10)):
                rospy.logerr("Failed to wait /tf2_buffer_server")
                self.fixed_frame_id = None

        self.pub_detect = self.advertise("~output", ObjectDetection, queue_size=1)
        self.pub_detect_nearest = self.advertise("~output/nearest", ObjectDetection, queue_size=1)

    def subscribe(self):
        self.subscribers = [
            message_filters.Subscriber("~input/boxes", BoundingBoxArray),
            message_filters.Subscriber("~input/classes", ClassificationResult),
        ]
        sync = message_filters.TimeSynchronizer(self.subscribers, self.queue_size)
        sync.registerCallback(self.on_result)

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()

    def align_box(self, box):
        ori = box.pose.orientation
        if ori.x < ori.y:
            q = T.quaternion_multiply(
                (ori.x, ori.y, ori.z, ori.w),
                T.quaternion_from_euler(0, 0, np.pi / 2.0))
            box.pose.orientation = Quaternion(*q)
            box.dimensions.x, box.dimensions.y = box.dimensions.y, box.dimensions.x
        return box

    def get_nearest(self, boxes, classes):
        nearest = {}
        for box, label, proba in zip(boxes.boxes, classes.label_names, classes.label_proba):
            if not label:
                continue
            if self.fixed_frame_id is not None:
                try:
                    ps = PoseStamped(header=box.header,
                                     pose=box.pose)
                    ps = self.tfl.transform(
                        ps, self.fixed_frame_id)
                    dist = np.sqrt(
                        ps.pose.position.x ** 2 +
                        ps.pose.position.y ** 2 +
                        ps.pose.position.z ** 2)
                except Exception as e:
                    rospy.logerr(str(e))
                    continue
            else:
                dist = np.sqrt(
                    box.pose.position.x ** 2 +
                    box.pose.position.y ** 2 +
                    box.pose.position.z ** 2)
            if label not in nearest:
                nearest[label] = (box, proba, dist)
            elif dist < nearest[label][2]:
                nearest[label] = (box, proba, dist)
        return nearest

    def publish_box_tf(self, box, label):
        pos, ori = (box.pose.position,
                    box.pose.orientation)
        t = TransformStamped()
        t.header = box.header
        t.child_frame_id = label
        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = pos.z
        t.transform.rotation.x = ori.x
        t.transform.rotation.y = ori.y
        t.transform.rotation.z = ori.z
        t.transform.rotation.w = ori.w
        self.tfb.sendTransform(t)

    def on_result(self, boxes, classes):
        msg = ObjectDetection()
        msg.header = boxes.header
        for box, label, proba in zip(boxes.boxes, classes.label_names, classes.label_proba):
            if not label:
                continue
            if self.align_box_pose:
                box = self.align_box(box)
            pose = Object6DPose()
            pose.pose = box.pose
            pose.reliability = proba
            pose.type = label
            msg.objects.append(pose)
        self.pub_detect.publish(msg)

        msg = ObjectDetection()
        msg.header = boxes.header
        nearest = self.get_nearest(boxes, classes)
        for label, (box, proba, dist) in nearest.items():
            if self.align_box_pose:
                box = self.align_box(box)
            pose = Object6DPose()
            pose.pose = box.pose
            pose.reliability = proba
            pose.type = label
            msg.objects.append(pose)
            if self.publish_tf:
                self.publish_box_tf(box, label)
        self.pub_detect_nearest.publish(msg)

if __name__ == '__main__':
    rospy.init_node("color_histogram_detector")
    detector = ColorHistogramDetector()
    rospy.spin()
