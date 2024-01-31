#!/usr/bin/env python

from collections import deque

from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
from statistics import mean
import rospy
from std_msgs.msg import Bool


class PeoplePoseAggregatedDetector(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        # self.pub = self.advertise(
        #     '~output/detected', Bool, queue_size=1)
        self.pub = self.advertise(
            '~output/aggregated', PeoplePose, queue_size=1)
        self.duration = rospy.get_param('~duration', 2.0)
        self.detected_ratio = rospy.get_param('~detected_ratio', 0.8)
        self.history = deque()

    def subscribe(self):
        self.start_time = rospy.Time.now()
        self.history = deque()
        self.sub = rospy.Subscriber(
            '~input',
            PeoplePoseArray,
            callback=self.callback,
            queue_size=1)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):
        peoplepose_msg = PeoplePose()
        self.history.append(msg)
        if (rospy.Time.now() - self.start_time).to_sec() < self.duration:
            rospy.logwarn('waiting duration time passed.')
            # self.pub.publish(Bool(data=False))
            return
        cur_time = rospy.Time.now()
        while len(self.history):
            if (cur_time - self.history[0].header.stamp).to_sec() > \
                    self.duration:
                self.history.popleft()
                continue
            break

        cnt = 0
        for msg in self.history:
            if len(msg.poses) > 0:
                cnt += 1
        if 1.0 * cnt / len(self.history) > self.detected_ratio:
            # self.pub.publish(Bool(data=True))
            averages = []
            for pose in msg.poses:
                average = mean(pose.scores)
                averages.append(average)
            max_score = max(averages)
            max_index = averages.index(max_score)
            if max_score > 0.7:
                peoplepose_msg.limb_names = msg.poses[max_index].limb_names
                # rospy.logwarn("{}".format(msg.poses[max_index].poses[0]))
                peoplepose_msg.poses = msg.poses[max_index].poses
                peoplepose_msg.scores = msg.poses[max_index].scores
                rospy.logwarn("published====================================")
            else:
                rospy.logwarn("score threshold")
            self.pub.publish(peoplepose_msg)
        else:
            # self.pub.publish(Bool(data=False))
            pass


if __name__ == '__main__':
    rospy.init_node('people_pose_aggregated_detector')
    cagg = PeoplePoseAggregatedDetector()  # noqa
    rospy.spin()
