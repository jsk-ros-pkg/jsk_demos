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


class PoseToLabanNode(object):

    def __init__(self):
        super(PoseToLabanNode, self).__init__()

        # self.pose_laban = self.advertise(
        #     '~output/pose',
        #     PeoplePoseArray, queue_size=1)
        # self.pub_img_compressed = self.advertise(
        #     '~output/viz/compressed',
        #     CompressedImage, queue_size=1)
        # self.skeleton_pub = self.advertise(
        #     '~output/skeleton', HumanSkeletonArray, queue_size=1)
        self.subscribe()

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '/people_pose_estimation/output/skeleton',
            # '~input',
            HumanSkeletonArray, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, skeleton_msg):
        print(skeleton_msg)
        # self.pose_pub.publish(people_pose_msg)
        # self.skeleton_pub.publish(skeleton_msgs)


if __name__ == '__main__':
    rospy.init_node('pose_to_laban')
    node = PoseToLabanNode()
    rospy.spin()
