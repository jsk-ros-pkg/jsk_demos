#!/usr/bin/env python

import cv_bridge
from jsk_recognition_msgs.msg import BoolStamped
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image


class FindHumanInMirror(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.mirror_label = rospy.get_param('~mirror_label', 1)
        self.score_thre = rospy.get_param('~score_threshold', 0.5)
        self.bridge = cv_bridge.CvBridge()
        self.no_sync = rospy.get_param('~no_sync', False)
        self.approximate_sync = rospy.get_param('~approximate_sync', False)
        self.ppl_msg = None
        self.pub_inside = self.advertise(
            '~output/inside_mirror', BoolStamped, queue_size=1)
        self.pub_outside = self.advertise(
            '~output/outside_mirror', BoolStamped, queue_size=1)

    def subscribe(self):
        if self.no_sync:
            sub_people_pose = rospy.Subscriber(
                '~input/people_pose_array', PeoplePoseArray,
                self._people_pose_cb, queue_size=1)
            sub_label = rospy.Subscriber(
                '~input/label', Image, self._label_cb, queue_size=1)
            self.subs = [sub_people_pose, sub_label]
        else:
            sub_people_pose = message_filters.Subscriber(
                '~input/people_pose_array', PeoplePoseArray, queue_size=1)
            sub_label = message_filters.Subscriber(
                '~input/label', Image,
                queue_size=1, buff_size=2**24)
            self.subs = [sub_people_pose, sub_label]
            if self.approximate_sync:
                queue_size = rospy.get_param('~queue_size', 10)
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    fs=self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self._sync_cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _people_pose_cb(self, ppl_msg):
        self.ppl_msg = ppl_msg

    def _label_cb(self, lbl_msg):
        if self.ppl_msg is not None:
            return self._cb(self.ppl_msg, lbl_msg)

    def _cb(self, ppl_msg, lbl_msg):
        rospy.loginfo('Start callback...')
        inside_msg = BoolStamped(header=lbl_msg.header, data=False)
        outside_msg = BoolStamped(header=lbl_msg.header, data=False)
        if not ppl_msg.poses:
            rospy.loginfo('No human found.')
            self.pub_inside.publish(inside_msg)
            self.pub_outside.publish(outside_msg)
            return

        lbl = self.bridge.imgmsg_to_cv2(lbl_msg)
        for person in ppl_msg.poses:
            # mean of [each_limb_score * int(bool(each_limb_is_in_mirror))]
            inside_score = np.mean([
                int((0 <= limb.position.x <= lbl_msg.width) and
                    (0 <= limb.position.y <= lbl_msg.height) and
                    lbl[int(limb.position.y),
                        int(limb.position.x)] == self.mirror_label) *
                person.scores[i]
                for i, limb in enumerate(person.poses)
            ])
            # mean of [each_limb_score * int(bool(each_limb_is_in_background))]
            outside_score = np.mean([
                int((0 <= limb.position.x <= lbl_msg.width) and
                    (0 <= limb.position.y <= lbl_msg.height) and
                    lbl[int(limb.position.y),
                        int(limb.position.x)] != self.mirror_label) *
                person.scores[i]
                for i, limb in enumerate(person.poses)
            ])
            rospy.loginfo(
                'inside score: {}, outside score: {}, threshold: {}'.format(
                    inside_score, outside_score, self.score_thre))
            if inside_score >= max(self.score_thre, outside_score):
                inside_msg.data = True
            if outside_score >= max(self.score_thre, inside_score):
                outside_msg.data = True

        self.pub_inside.publish(inside_msg)
        self.pub_outside.publish(outside_msg)
        return


if __name__ == '__main__':
    rospy.init_node('find_human_in_mirror')
    app = FindHumanInMirror()
    rospy.spin()
