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
        self.pub = self.advertise('~output', BoolStamped, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_people_pose = message_filters.Subscriber(
            '~input/people_pose_array', PeoplePoseArray,
            queue_size=1, buff_size=2**24)
        sub_label = message_filters.Subscriber(
            '~input/label', Image,
            queue_size=1, buff_size=2**24)
        self.subs = [sub_people_pose, sub_label]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb(self, ppl_msg, lbl_msg):
        rospy.loginfo('Start callback...')
        out_msg = BoolStamped(header=lbl_msg.header, data=False)
        if not ppl_msg.poses:
            rospy.loginfo('No human found.')
            self.pub.publish(out_msg)
            return

        lbl = self.bridge.imgmsg_to_cv2(lbl_msg)
        for person in ppl_msg.poses:
            # mean of [each_limb_score * int(bool(each_limb_is_in_lbl))]
            score = np.mean([
                int((0 <= limb.position.x <= lbl_msg.width) and
                    (0 <= limb.position.y <= lbl_msg.height) and
                    lbl[int(limb.position.y),
                        int(limb.position.x)] == self.mirror_label) *
                person.scores[i]
                for i, limb in enumerate(person.poses)
            ])
            rospy.loginfo('score: {}, threshold: {}'.format(
                score, self.score_thre))
            if score >= self.score_thre:
                out_msg.data = True
                self.pub.publish(out_msg)
                return
        self.pub.publish(out_msg)
        return


if __name__ == '__main__':
    rospy.init_node('find_human_in_mirror')
    app = FindHumanInMirror()
    rospy.spin()
