#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import message_filters
import tf2_ros
from geometry_msgs.msg import TransformStamped
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from posedetection_msgs.msg import ObjectDetection, Object6DPose


class ColorHistogramDetector(ConnectionBasedTransport):
    def __init__(self):
        super(ColorHistogramDetector, self).__init__()
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.publish_tf = rospy.get_param("~publish_tf", False)
        if self.publish_tf:
            self.tfb = tf2_ros.TransformBroadcaster()
        self.pub_detect = self.advertise("~output", ObjectDetection, queue_size=1)

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

    def on_result(self, boxes, classes):
        rospy.logwarn("on_result")
        msg = ObjectDetection()
        msg.header = boxes.header
        for box, label, prob in zip(boxes.boxes, classes.label_names, classes.label_proba):
            if not label:
                continue
            pose = Object6DPose()
            pose.pose = box.pose
            pose.reliability = prob
            pose.type = label
            msg.objects.append(pose)
            if self.publish_tf:
                t = TransformStamped()
                t.header = box.header
                t.child_frame_id = label
                t.transform.translation.x = box.pose.position.x
                t.transform.translation.y = box.pose.position.y
                t.transform.translation.z = box.pose.position.z
                t.transform.rotation.x = box.pose.orientation.x
                t.transform.rotation.y = box.pose.orientation.y
                t.transform.rotation.z = box.pose.orientation.z
                t.transform.rotation.w = box.pose.orientation.w
                self.tfb.sendTransform(t)
        self.pub_detect.publish(msg)

if __name__ == '__main__':
    rospy.init_node("color_histogram_detector")
    detector = ColorHistogramDetector()
    rospy.spin()
