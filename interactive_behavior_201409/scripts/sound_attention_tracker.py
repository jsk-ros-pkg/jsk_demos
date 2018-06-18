#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import angles
import copy
import math
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from jsk_topic_tools import ConnectionBasedTransport
from sklearn.decomposition import PCA
import tf.transformations as T

from interactive_behavior_201409.msg import Attention
from jsk_hark_msgs.msg import HarkPower
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class SoundAttentionTracker(ConnectionBasedTransport):
    def __init__(self):
        super(SoundAttentionTracker, self).__init__()
        self.use_hark = rospy.get_param("~use_hark", False)
        if self.use_hark:
            self.bias = rospy.get_param("~bias", 25)
            self.gradient = rospy.get_param("~gradient", 0.15)
            self.scale = rospy.get_param("~scale", 1.0)
            self.mean_threshold = rospy.get_param("~mean_threshold", 0.05)
            self.covariance_threshold = rospy.get_param("~covariance_threshold", 0.25)
        else:
            self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom_combined")
            self.angle_threshold = rospy.get_param("~angle_threshold", 20)  # [deg]
            self.average_threshold = rospy.get_param("~average_threshold", 3)  # [num]
            self.speech_threshold = rospy.get_param("~speech_threshold", False)
            self.sensor_frame_id = None
            self.is_speeching = False
            self.doa_history = []

        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        if not self.tfl.wait_for_server(rospy.Duration(10)):
            rospy.logerr("Failed to wait for /tf2_buffer_server")
            self.tfl = tf2_ros.Buffer()

        self.pub_axes = self.advertise("~output/axes", PoseStamped, queue_size=1)
        self.pub_attention = self.advertise("~output", Attention, queue_size=1)

    def subscribe(self):
        if self.use_hark:
            self.subscribers = [
                rospy.Subscriber("/HarkPower", HarkPower,
                                 self.hark_callback, queue_size=1)
            ]
        else:
            self.subscribers = [
                rospy.Subscriber("/sound_localization", PoseStamped,
                                 self.doa_callback, queue_size=1),
                rospy.Subscriber("/is_speeching", Bool,
                                 self.is_speeching_cb, queue_size=1),
            ]

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()

    def doa_callback(self, msg):
        if self.speech_threshold and not self.is_speeching:
            rospy.logdebug("No speech detected. skipping DOA")
            return

        try:
            self.sensor_frame_id = msg.header.frame_id
            t = self.tfl.transform(msg, self.odom_frame_id, timeout=rospy.Duration(5.0))
            self.doa_history.append(t)
            self.doa_history = self.doa_history[-self.average_threshold:]
        except Exception as e:
            rospy.logerr(e)

        if len(self.doa_history) < self.average_threshold:
            return

        avg = 0.0
        for p in self.doa_history:
            q = p.pose.orientation
            e = T.euler_from_quaternion((q.x, q.y, q.z, q.w))
            avg += e[2]
        avg /= len(self.doa_history)

        last = self.doa_history[-1]
        q = last.pose.orientation
        e = T.euler_from_quaternion((q.x, q.y, q.z, q.w))
        last_angle = e[2]
        diff = angles.shortest_angular_distance(avg, last_angle)
        if abs(diff) < math.radians(self.angle_threshold):
            axes_msg = self.tfl.transform(last, self.sensor_frame_id, timeout=rospy.Duration(5.0))
            q = axes_msg.pose.orientation
            e = T.euler_from_quaternion((q.x, q.y, q.z, q.w))
            yaw = e[2]
            axes_msg.pose.position.x = 1.0 * np.cos(yaw)
            axes_msg.pose.position.y = 1.0 * np.sin(yaw)
            self.pub_axes.publish(axes_msg)

            msg = Attention(header=axes_msg.header, level=Attention.LEVEL_IMPORTANT)
            msg.type = "sound"
            msg.target = axes_msg.pose
            rospy.loginfo("Attention!")
            self.pub_attention.publish(msg)
        else:
            rospy.logwarn("Ambiguous DOA: %s" % diff)

    def is_speeching_cb(self, msg):
        self.is_speeching = msg.data

    def hark_callback(self, msg):
        biased_2d_power = list()
        n = len(msg.powers)
        for i, p in enumerate(msg.powers):
            biased_power = self.gradient * (p - self.bias)
            if biased_power <= 0.0:
                biased_power = 0.001
            biased_power *= self.scale
            biased_2d_power.append(
                [biased_power * np.cos(i * 2.0 * np.pi / n - np.pi),
                 biased_power * np.sin(i * 2.0 * np.pi / n - np.pi)])
        biased_2d_power= np.array(biased_2d_power)

        pca = PCA(n_components=2)
        pca.fit(biased_2d_power)

        yaw = np.arctan2(pca.components_[0][1], pca.components_[0][0])
        q = T.quaternion_from_euler(0, 0, yaw)

        axes_msg = PoseStamped(header=msg.header)
        axes_msg.pose.position.x = pca.mean_[0] + 1.0 * np.cos(yaw)
        axes_msg.pose.position.y = pca.mean_[1] + 1.0 * np.sin(yaw)
        axes_msg.pose.orientation.x = q[0]
        axes_msg.pose.orientation.y = q[1]
        axes_msg.pose.orientation.z = q[2]
        axes_msg.pose.orientation.w = q[3]
        self.pub_axes.publish(axes_msg)

        mean = np.linalg.norm(pca.mean_)
        cov = np.linalg.norm(pca.get_covariance())
        rospy.loginfo("mean %f cov %f" % (mean, cov))
        if mean > self.mean_threshold and cov < self.covariance_threshold:
            msg = Attention(header=msg.header, level=Attention.LEVEL_IMPORTANT)
            msg.type = "sound"
            msg.target = axes_msg.pose
            rospy.loginfo("Attention!")
            self.pub_attention.publish(msg)


if __name__ == '__main__':
    rospy.init_node("sound_attention_tracker")
    t = SoundAttentionTracker()
    rospy.spin()
