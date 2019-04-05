#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import functools
from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
from threading import Lock
import tf2_ros
import tf2_geometry_msgs
from sklearn.covariance import EllipticEnvelope

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
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


def pose_position_norm(p):
    pos = p.pose.position
    return np.linalg.norm([pos.x, pos.y, pos.z])


def remove_outlier_limbs(person, mean_thresh=0.5, score_thresh=0.4):
    if len(person.poses) < 2:
        return person
    xs = np.asarray([[p.position.z] for p in person.poses])
    ys = EllipticEnvelope().fit(xs).predict(xs)
    mean = np.mean(xs[ys > 0])
    neg = np.mean(xs[ys < 0])

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


def people_msg_to_pose(people_msg, min_score):
    limbs = {}
    for limb, pose, score in zip(people_msg.limb_names,
                                 people_msg.poses,
                                 people_msg.scores):
        if score > min_score:
            limbs[limb] = pose

    body = [limbs[l] for l in ["Neck", "LHip", "RHip", "LShoulder", "RShoulder"] if l in limbs]
    head = [limbs[l] for l in ["Nose", "LEye", "REye"] if l in limbs]
    foot = [limbs[l] for l in ["LKnee", "RKnee"] if l in limbs]
    arm = [limbs[l] for l in ["LWrist", "LElbow", "RWrist", "RElbow"] if l in limbs]

    def avg(poses):
        x = np.mean([p.position.x for p in poses])
        y = np.mean([p.position.y for p in poses])
        z = np.mean([p.position.z for p in poses])
        return np.array([x, y, z])

    ret = np.array([0.0, 0.0, 0.0])
    parts = [avg(body), avg(head), avg(foot), avg(arm)]
    weights = [5.0, 2.0, 2.0, 1.0]
    ws = []
    for w, p in zip(weights, parts):
        if not np.isnan(p).any():
            ret += w * p
            ws.append(w)
    ret = ret / sum(ws)

    pose = Pose()
    pose.position.x = ret[0]
    pose.position.y = ret[1]
    pose.position.z = ret[2]
    pose.orientation.w = 1.0
    return pose


DECLARED_NAMES = set()
NAME_COUNTER = 0
def generate_name():
    global DECLARED_NAMES
    global NAME_COUNTER
    try:
        import names
        while True:
            name = names.get_first_name()
            if name not in DECLARED_NAMES:
                DECLARED_NAMES.add(name)
                return name
    except ImportError:
        name = 'person{}'.format(NAME_COUNTER)
        NAME_COUNTER += 1
        return name


@functools.total_ordering
class Person(object):
    names = set()
    sensor_frame = 'kinect_head_rgb_optical_frame'
    fixed_frame = 'odom_combined'
    near_threshold = 0.5
    timeout_threshold = 5.0
    valid_threshold = 0.5
    transform_listener = None

    def __init__(self, pose, confidence, name=None):
        super(Person, self).__init__()

        assert isinstance(pose, PoseStamped)

        if pose.header.frame_id.startswith('/'):
            pose.header.frame_id = pose.header.frame_id[1:]
        if pose.header.frame_id != Person.fixed_frame:
            try:
                tfl = Person.get_tfl()
                pose = tfl.transform(pose, Person.fixed_frame)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
            except tf2_ros.TransformException as e:
                rospy.logerr(e)
                confidence = 0.0
        norm = pose_position_norm(pose)
        if np.isnan(norm) or norm == 0:
            rospy.logwarn("Pose norm is nan or zero")
            confidence = 0.0

        self.pose = pose
        self.confidence = confidence

        if name is None:
            self.name = generate_name()
        else:
            assert isinstance(name, str)
            self.name = name

    @classmethod
    def set_fixed_frame(cls, frame):
        cls.fixed_frame = frame

    @classmethod
    def set_sensor_frame(cls, frame):
        cls.sensor_frame = frame

    @classmethod
    def set_near_threshold(cls, value):
        cls.near_threshold = value

    @classmethod
    def get_tfl(cls):
        if cls.transform_listener is None:
            tfl = tf2_ros.BufferClient('/tf2_buffer_server')
            if not tfl.wait_for_server(rospy.Duration(10)):
                rospy.logerr("Failed to wait for /tf2_buffer_server")
                tfl = tf2_ros.Buffer()
            cls.transform_listener = tfl
        return cls.transform_listener

    def __str__(self):
        return self.name

    def __eq__(self, p):
        return self.name == p.name

    def __ne__(self, p):
        return not self.__eq__(p)

    def __lt__(self, p):
        return self.attention < p.attention

    def is_near(self, p):
        if self.pose.header.frame_id != p.pose.header.frame_id:
            rospy.logerr("Compared poses with different frame_id")
            return False
        if pose_position_norm(self.pose) == 0:
            rospy.logerr("norm(self.pose) == 0")
            return False
        if pose_position_norm(p.pose) == 0:
            rospy.logerr("norm(p.pose) == 0")
            return False
        dist = pose_distance(self.pose, p.pose)
        return dist < Person.near_threshold

    @property
    def attention(self):
        age = (rospy.Time.now() - self.pose.header.stamp).to_sec()
        age_factor = Person.timeout_threshold - min(Person.timeout_threshold, age)
        age_factor /= Person.timeout_threshold
        return self.confidence * age_factor

    @property
    def is_valid(self):
        return self.attention > Person.valid_threshold


class Comrades(object):
    def __init__(self):
        self.group = []
        self.lock = Lock()
        self.last_target = None

    def add(self, person):
        with self.lock:
            dups = [p for p in self.group if p.is_near(person)]
            if dups:
                dups.sort()
                name = dups[0].name
                for d in dups:
                    self.group.remove(d)
                dups.append(person)
                dups.sort(reverse=True)
                new_person = dups[0]
                new_person.name = name
                self.group.append(new_person)
            else:
                self.group.append(person)

    def purge(self):
        self.group = [p for p in self.group if p.is_valid]

    def get_target(self):
        with self.lock:
            self.purge()

            if not self.group:
                return None

            self.group.sort(reverse=True)

            target = None
            # search last target
            if self.last_target is not None:
                ps = [p for p in self.group if p.name == self.last_target]
                if ps:
                    target = ps[0]
            if target is None:
                target = self.group[0]

            self.last_target = target.name
            return target

    def show(self):
        with self.lock:
            self.purge()
            self.group.sort(reverse=True)
            for i, p in enumerate(self.group):
                rospy.loginfo('#{} Name: {}, Norm: {}, Attn: {}'.format(i, p.name, pose_position_norm(p.pose), p.attention))


class PeopleAttentionTracker(ConnectionBasedTransport):
    def __init__(self):
        super(PeopleAttentionTracker, self).__init__()

        self.enable_face = rospy.get_param('~enable_face', False)
        self.fixed_frame = rospy.get_param('~fixed_frame', 'odom_combined')
        self.sensor_frame = rospy.get_param('~sensor_frame', 'kinect_head_rgb_optical_frame')
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)

        self.comrades = Comrades()

        self.pub_timer = None
        self.attention_pub = self.advertise(
            "~output", Attention, queue_size=1)

    def subscribe(self):
        if self.enable_face:
            approximate_sync = rospy.get_param("~approximate_sync", True)
            queue_size = rospy.get_param("~queue_size", 100)
            slop = rospy.get_param("~slop", 0.1)
            self.subscribers = [
                MF.Subscriber("~input/face_pose",
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
            sync.registerCallback(self.callback)
        else:
            self.subscribers = [
                rospy.Subscriber('~input/people_pose', PeoplePoseArray,
                                 self.people_callback, queue_size=1)
            ]

        try:
            self.pub_timer = rospy.Timer(
                rospy.Duration(1.0 / self.publish_rate), self.timer_callback, reset=True)
        except:
            self.pub_timer = rospy.Timer(
                rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        rospy.loginfo("subscribed")

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()
        self.pub_timer.shutdown()
        rospy.loginfo('unsubscribed')

    def timer_callback(self, event=None):
        # self.comrades.show()
        person = self.comrades.get_target()
        if person is None:
            return

        msg = Attention(header=person.pose.header, target=person.pose.pose,
                        type='people', level=Attention.LEVEL_IMPORTANT)
        self.attention_pub.publish(msg)


    def people_callback(self, people):
        header = people.header
        poses = [people_msg_to_pose(p, 0.5) for p in people.poses]
        scores = [max(p.scores) if p.scores else None for p in people.poses]

        for pose, score in zip(poses, scores):
            if score is None:
                continue
            pose = PoseStamped(header=header, pose=pose)
            person = Person(pose, score)
            self.comrades.add(person)

    def callback(self, poses, faces):
        header = poses.header
        for pose, face in zip(poses.poses, faces.faces):
            name = face.label
            score = face.confidence
            pose = PoseStamped(header=header, pose=pose)
            person = Person(pose, score, name)
            self.comrades.add(person)



if __name__ == '__main__':
    rospy.init_node("people_attention_tracker")
    t = PeopleAttentionTracker()
    rospy.spin()
