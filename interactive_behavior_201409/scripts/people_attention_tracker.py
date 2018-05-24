#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import angles
from collections import defaultdict
from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
import tf.transformations as T
import tf2_ros
import tf2_geometry_msgs
from jsk_interactive_behavior import get_tfl

from geometry_msgs.msg import PoseArray, PoseStamped
from jsk_recognition_msgs.msg import ClassificationResult
from opencv_apps.msg import FaceArrayStamped
from jsk_interactive_behavior.msg import Attention


def pose_distance(p1, p2):
    dp  = (p1.pose.position.x - p2.pose.position.x) ** 2
    dp += (p1.pose.position.y - p2.pose.position.y) ** 2
    dp += (p1.pose.position.z - p2.pose.position.z) ** 2

    dt = np.abs((p1.header.stamp - p2.header.stamp).to_sec())
    if dt == 0:
        dt = 1.0
    return np.sqrt(dp) / dt


class PeopleAttentionTracker(ConnectionBasedTransport):
    def __init__(self):
        super(PeopleAttentionTracker, self).__init__()

        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "odom_combined")
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.25)
        self.timeout_threshold = rospy.get_param("~timeout_threshold", 5.0)
        self.face_recognition_threshold = rospy.get_param("~face_recognition_threshold", 4300.0)
        self.face_pose_threshold = rospy.get_param("~face_pose_threshold", 0.2)
        self.attention_score_threshold = rospy.get_param("~attention_score_threshold", 30.0)
        self.attention_timeout_threshold = rospy.get_param("~attention_timeout_threshold", 3.0)
        self.unknown_name_prefix = rospy.get_param("~unknown_name_prefix", "unknown")

        self.tfl = get_tfl()

        self.attention_pub = self.advertise(
            "~output", Attention, queue_size=1)
        self.people_pub = self.advertise(
            "~output/people", ClassificationResult, queue_size=1)
        self.attention_class_pub = self.advertise(
            "~output/attention", ClassificationResult, queue_size=1)

    def subscribe(self):
        self.name_counter = defaultdict(int)
        self.people = dict()
        self.last_received = None
        self.attentions = defaultdict(list)

        approximate_sync = rospy.get_param("~approximate_sync", False)
        queue_size = rospy.get_param("~queue_size", 100)
        slop = rospy.get_param("~slop", 0.1)

        self.subscribers = [
            MF.Subscriber("~input/pose",
                          PoseArray, queue_size=1),
            MF.Subscriber("~input/face",
                          FaceArrayStamped, queue_size=1),
        ]

        if approximate_sync:
            sync = MF.ApproximateTimeSynchronizer(self.subscribers,
                                                  queue_size=queue_size,
                                                  slop=slop)
        else:
            sync = MF.TimeSynchronizer(self.subscribers,
                                       queue_size=queue_size)
        sync.registerCallback(self.msg_callback)
        rospy.loginfo("subscribed")

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()
        rospy.loginfo("unsubscribed")

    def msg_callback(self, poses, faces):
        # pop old poses
        stamp = poses.header.stamp
        if self.last_received and self.last_received < stamp:
            new_poses = dict()
            for n, p in self.people.items():
                if (stamp - p.header.stamp).to_sec() < self.timeout_threshold:
                    new_poses[n] = p
                else:
                    rospy.logdebug("popped: %s" % n)
            self.people = new_poses

        people_msg = ClassificationResult(header=poses.header)
        people_msg.classifier = "people_tracker"

        attention_cls_msg = ClassificationResult(header=poses.header)
        attention_cls_msg.classifier = "attention_estimator"

        for pose, face in zip(poses.poses, faces.faces):
            rot = pose.orientation
            rot = T.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
            pose = PoseStamped(header=poses.header, pose=pose)

            # ensure transformation is from fixed frame
            if not pose.header.frame_id != self.fixed_frame_id:
                try:
                    pose = self.tfl.transform(pose, self.fixed_frame_id)
                except:
                    continue

            # find near poses
            dups = list()
            for n, p in self.people.items():
                d = pose_distance(pose, p)
                if d < self.distance_threshold:
                    dups.append((n, p))
                    del self.people[n]

            # merge into one name
            if face.label:
                name = face.label
                proba = (self.face_recognition_threshold - face.confidence) / self.face_recognition_threshold
                rospy.logdebug("%s -> %s (label)" % ([n[0] for n in dups], name))
            elif dups:
                known_names = filter(lambda n: not n[0].startswith(self.unknown_name_prefix), dups)
                if known_names:
                    name = known_names[0][0]
                else:
                    name = dups[0][0]
                proba = 0.5
                rospy.logdebug("%s -> %s" % ([n[0] for n in dups], name))
            else:
                name = self.generate_name()
                rospy.logdebug("None -> %s (new)" % name)
                proba = 0.5

            # fill class msg
            self.people[name] = pose
            people_msg.label_names.append(name)
            people_msg.label_proba.append(proba)

            # count attention
            rot_dist = angles.shortest_angular_distance(rot[0], -np.pi)
            if rot_dist < self.face_pose_threshold:
                self.attentions[name] += [stamp]
            attentions = filter(
                lambda t: (stamp-t).to_sec()<=self.attention_timeout_threshold,
                sorted(self.attentions[name]))
            self.attentions[name] = attentions
            if len(attentions) < 2:
                score = 0.0
            else:
                score = 1.0 * len(attentions) / self.attention_score_threshold
                rospy.logdebug("attention: %f" % score)
            score = min(1.0, max(0.0, score))
            attention_cls_msg.label_names.append(name)
            attention_cls_msg.label_proba.append(score)

        people_msg.target_names = list(set(people_msg.label_names))
        attention_cls_msg.target_names = list(set(attention_cls_msg.label_names))

        self.people_pub.publish(people_msg)
        self.attention_class_pub.publish(attention_cls_msg)

        normal_attention, normal_max_score = None, 0.0
        important_attention, important_max_score = None, 0.0
        for i, score in enumerate(attention_cls_msg.label_proba):
            pose = self.people[attention_cls_msg.label_names[i]]
            if score < 0.3 and score > normal_max_score:
                normal_attention = Attention(
                    header=pose.header, type="people", level=Attention.LEVEL_NORMAL)
                normal_attention.target = pose.pose
                normal_max_score = score
            elif 0.3 <= score and score > important_max_score:
                important_attention = Attention(
                    header=pose.header, type="people", level=Attention.LEVEL_IMPORTANT)
                important_attention.target = pose.pose
                important_max_score = score

        if normal_attention is not None:
            self.attention_pub.publish(normal_attention)
        if important_attention is not None:
            self.attention_pub.publish(important_attention)

        self.last_received = stamp

    def generate_name(self, base=None):
        if base is None:
            base = self.unknown_name_prefix
        i = self.name_counter[base]
        self.name_counter[base] += 1
        return base + str(i)


if __name__ == '__main__':
    rospy.init_node("people_attention_tracker")
    t = PeopleAttentionTracker()
    rospy.spin()
