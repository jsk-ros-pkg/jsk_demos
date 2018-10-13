#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import angles
from collections import defaultdict, Counter
from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
import tf.transformations as T
import tf2_ros
import tf2_geometry_msgs
from sklearn.covariance import EllipticEnvelope

from geometry_msgs.msg import PoseArray, PoseStamped
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import PeoplePoseArray
from opencv_apps.msg import FaceArrayStamped
from interactive_behavior_201409.msg import Attention


def pose_distance(p1, p2):
    dp  = (p1.pose.position.x - p2.pose.position.x) ** 2
    dp += (p1.pose.position.y - p2.pose.position.y) ** 2
    dp += (p1.pose.position.z - p2.pose.position.z) ** 2

    dt = np.abs((p1.header.stamp - p2.header.stamp).to_sec())
    try:
        return np.sqrt(dp) / dt
    except:
        return np.sqrt(dp)


def remove_outlier_limbs(person, mean_thresh=0.5, score_thresh=0.4):
    if len(person.poses) < 2:
        return person
    xs = np.asarray([[p.position.z] for p in person.poses])
    ys = EllipticEnvelope().fit(xs).predict(xs)
    mean = np.mean(xs[ys > 0])
    neg = np.mean(xs[ys < 0])

    rospy.loginfo((mean, neg))
    if neg < mean and abs(mean - neg) > mean_thresh:
        mean, neg = neg, mean

    limbs, poses, scores = [], [], []
    for i in range(len(person.poses)):
        if abs(person.poses[i].position.z - mean) < mean_thresh and\
           person.scores[i] > score_thresh:
            limbs.append(person.limb_names[i])
            poses.append(person.poses[i])
            scores.append(person.scores[i])
        else:
            rospy.logdebug("Dropped %s" % person.limb_names[i])

    person.poses = poses
    person.limb_names = limbs
    person.scores = scores

    return person


class PeopleAttentionTracker(ConnectionBasedTransport):
    def __init__(self):
        super(PeopleAttentionTracker, self).__init__()

        # parameters
        self.enable_face_identification = rospy.get_param(
            "~enable_face_identification", False)
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "odom_combined")
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.5)
        self.timeout_threshold = rospy.get_param("~timeout_threshold", 5.0)
        self.face_recognition_threshold = rospy.get_param("~face_recognition_threshold", 4300.0)
        self.face_pose_threshold = rospy.get_param("~face_pose_threshold", 0.2)
        self.attention_score_threshold = rospy.get_param("~attention_score_threshold", 30.0)
        self.attention_timeout_threshold = rospy.get_param("~attention_timeout_threshold", 3.0)
        self.unknown_name_prefix = rospy.get_param("~unknown_name_prefix", "person")

        # variables
        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        if not self.tfl.wait_for_server(rospy.Duration(10)):
            rospy.logerr("Failed to wait for /tf2_buffer_server")
            self.tfl = tf2_ros.Buffer()
        self.limbs = ["Nose", "Neck", "RShoulder", "LShoulder", "RHip", "LHip",]

        # advertise
        self.attention_pub = self.advertise(
            "~output", Attention, queue_size=1)
        self.pose_pub = self.advertise(
            "~output/pose", PoseArray, queue_size=1)
        self.people_pub = self.advertise(
            "~output/people", ClassificationResult, queue_size=1)
        self.attention_class_pub = self.advertise(
            "~output/attention", ClassificationResult, queue_size=1)

    def subscribe(self):
        self.last_received = None
        self.last_tracked_name = None
        self.name_counter = defaultdict(int)
        self.people = dict()
        self.attentions = defaultdict(list)

        approximate_sync = rospy.get_param("~approximate_sync", True)
        queue_size = rospy.get_param("~queue_size", 100)
        slop = rospy.get_param("~slop", 0.1)

        if self.enable_face_identification:
            self.subscribers = [
                MF.Subscriber("~input/face_pose",
                              PoseArray, queue_size=1),
                MF.Subscriber("~input/face",
                              FaceArrayStamped, queue_size=1),
            ]
        else:
            self.subscribers = [
                MF.Subscriber("~input/people_pose",
                              PeoplePoseArray, queue_size=1),
            ]

        if approximate_sync:
            sync = MF.ApproximateTimeSynchronizer(self.subscribers,
                                                  queue_size=queue_size,
                                                  slop=slop)
        else:
            sync = MF.TimeSynchronizer(self.subscribers,
                                       queue_size=queue_size)
        sync.registerCallback(self.callback)
        rospy.loginfo("subscribed")

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()
        rospy.loginfo("unsubscribed")

    def pop_old_cache(self, stamp):
        # pop old poses
        if self.last_received and self.last_received < stamp:
            new_poses = dict()
            for n, p in self.people.items():
                if (stamp - p.header.stamp).to_sec() < self.timeout_threshold:
                    new_poses[n] = p
                else:
                    rospy.loginfo("popped: %s" % n)
            self.people = new_poses
        self.last_received = stamp

    def callback(self, poses, face_ids=None):
        stamp = poses.header.stamp
        self.pop_old_cache(stamp)
        #
        if poses.header.frame_id.startswith("/"):
            poses.header.frame_id = poses.header.frame_id[1:]
        #
        try:
            if self.enable_face_identification:
                self.callback_with_id(poses, face_ids)
            else:
                self.callback_without_id(poses)
        except Exception as e:
            rospy.logerr(e)
            import traceback
            rospy.logerr(traceback.format_exc())


    def callback_without_id(self, poses):
        stamp = poses.header.stamp

        for ppose in poses.poses:
            pose = self.get_people_pos(ppose)
            if pose is None:
                continue
            if poses.header.frame_id != self.fixed_frame_id:
                try:
                    ps = PoseStamped(header=poses.header, pose=pose)
                    pose = self.tfl.transform(ps, self.fixed_frame_id)
                except Exception as e:
                    rospy.logerr(e)
                    continue
            else:
                pose = PoseStamped(header=poses.header, pose=pose)

            # fix orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            # find near poses
            dups = list()
            for n, p in self.people.items():
                d = pose_distance(pose, p)
                if d < self.distance_threshold:
                    dups.append((n, p))
                    del self.people[n]
                    continue

            name = None
            if dups:
                name = Counter([n for n, p in dups]).most_common(1)
                if name:
                    name = name[0][0]
                else:
                    name = None
            if name is None:
                name = self.generate_name()

            self.people[name] = pose

        if not self.people:
            return
        header = self.people.values()[0].header
        header.stamp = stamp
        pub_msg = Attention(
            header=header, type="people", level=Attention.LEVEL_IMPORTANT)
        if self.last_tracked_name and self.last_tracked_name in self.people:
            pose = self.people[self.last_tracked_name]
            pub_msg.target = pose.pose
        elif self.people.values():
            name, pose = self.people.items()[0]
            pub_msg.target = pose.pose
            rospy.loginfo("%s -> %s" % (self.last_tracked_name, name))
            self.last_tracked_name = name
        self.attention_pub.publish(pub_msg)

        # pose
        pub_msg = PoseArray(header=header)
        for name, pose in self.people.items():
            pub_msg.poses.append(pose.pose)
        self.pose_pub.publish(pub_msg)

        # attention
        pub_msg = ClassificationResult(header=header)
        pub_msg.classifier = "people_tracker"
        people = self.people.items()
        names = [p[0] for p in people]
        target_names = list(set(names))
        pub_msg.target_names = target_names
        pub_msg.label_names = names
        pub_msg.labels = [target_names.index(n) for n in names]
        pub_msg.label_proba = [0.5 for n in names]
        self.attention_class_pub.publish(pub_msg)

    def get_people_pos(self, person):
        try:
            person = remove_outlier_limbs(person)
        except:
            return None
        for limb in self.limbs:
            try:
                idx = person.limb_names.index(limb)
                return person.poses[idx]
            except ValueError:
                continue

    def callback_with_id(self, poses, faces):
        stamp = poses.header.stamp

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
