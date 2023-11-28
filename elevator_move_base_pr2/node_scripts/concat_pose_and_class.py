#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
from geometry_msgs.msg import PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ClassificationResult
from posedetection_msgs.msg import ObjectDetection
from posedetection_msgs.msg import Object6DPose


class ConcatPoseAndClass(ConnectionBasedTransport):

    def __init__(self):
        super(ConcatPoseAndClass, self).__init__()
        self.frame_id = rospy.get_param('~frame_id', "base_footprint")
        self.pub_detect = self.advertise(
            "~output", ObjectDetection, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param("~queue_size", 100)
        approximate_sync = rospy.get_param('~approximate_sync', False)
        self.subscribers = [
            message_filters.Subscriber("~input/poses", PoseArray),
            message_filters.Subscriber("~input/classes", ClassificationResult),
        ]
        if approximate_sync:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subscribers, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                self.subscribers, queue_size)
        sync.registerCallback(self.cb)

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()

    def cb(self, poses, classes):
        msg = ObjectDetection()
        msg.header = poses.header
        for pose, label, prob in zip(
                poses.poses,
                classes.label_names,
                classes.label_proba):
            if not label:
                continue

            object_pose = Object6DPose()
            object_pose.pose = pose
            msg.header.frame_id = self.frame_id
            object_pose.reliability = prob
            object_pose.type = label
            msg.objects.append(object_pose)
        self.pub_detect.publish(msg)


if __name__ == '__main__':
    rospy.init_node("concat_pose_and_class")
    detector = ConcatPoseAndClass()
    rospy.spin()
