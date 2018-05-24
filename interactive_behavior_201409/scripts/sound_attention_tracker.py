#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
import numpy as np
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from sklearn.decomposition import PCA
import tf.transformations as T

from interactive_behavior_201409.msg import Attention
from jsk_hark_msgs.msg import HarkPower
from geometry_msgs.msg import PoseStamped


class SoundAttentionTracker(ConnectionBasedTransport):
    def __init__(self):
        super(SoundAttentionTracker, self).__init__()
        self.bias = rospy.get_param("~bias", 25)
        self.gradient = rospy.get_param("~gradient", 0.15)
        self.scale = rospy.get_param("~scale", 1.0)
        self.smooth = rospy.get_param("~smooth", 3)
        self.mean_threshold = rospy.get_param("~mean_threshold", 0.05)
        self.covariance_threshold = rospy.get_param("~covariance_threshold", 0.25)
        self.pub_attention = self.advertise("~output", Attention, queue_size=1)
        self.pub_raw = self.advertise("~output/raw_power", HarkPower, queue_size=1)
        self.pub_axes = self.advertise("~output/axes", PoseStamped, queue_size=1)
        self.powers_num = 100
        self.finders = []

    def subscribe(self):
        self.sub_hark = rospy.Subscriber(
            "/HarkPower", HarkPower, self.callback, queue_size=1)

    def unsubscribe(self):
        self.sub_hark.unregister()

    def callback(self, msg):
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

    def callback_old(self, msg):
        if len(msg.powers) == 0 or msg.powers[0] == 0.0:
            return

        while len(self.finders) < msg.directions:
            self.finders.append(changefinder.ChangeFinder(r=0.1, order=1, smooth=3))

        raw_msg = copy.copy(msg)
        raw_msg.directions = int(msg.directions / float(self.smooth))
        raw_msg.data_bytes = 4 * raw_msg.directions
        biased_powers = []
        for i in range(raw_msg.directions):
            p = 0.0
            for j in range(self.smooth):
                p += msg.powers[i*self.smooth+j]
            p /= self.smooth
            biased_power = self.gradient * (p - self.bias)
            if biased_power <= 0.0:
                biased_power = 0.001
            biased_power *= self.scale
            biased_powers.append(biased_power)
        raw_msg.powers = biased_powers
        self.pub_raw.publish(raw_msg)

        ch_msg = copy.copy(raw_msg)
        changes = []
        for c, p in zip(self.finders, biased_powers):
            changes.append(c.update(p))
        ch_msg.powers = changes
        self.pub_change.publish(ch_msg)
        rospy.loginfo(max(changes))

if __name__ == '__main__':
    rospy.init_node("sound_attention_tracker")
    t = SoundAttentionTracker()
    rospy.spin()
