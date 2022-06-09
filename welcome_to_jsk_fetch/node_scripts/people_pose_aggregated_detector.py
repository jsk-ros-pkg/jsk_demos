#!/usr/bin/env python

from collections import deque

from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from std_msgs.msg import Bool


class PeoplePoseAggregatedDetector(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pub = self.advertise(
            '~output/detected', Bool, queue_size=1)
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
        self.history.append(msg)
        if (rospy.Time.now() - self.start_time).to_sec() < self.duration:
            rospy.logwarn('waiting duration time passed.')
            self.pub.publish(Bool(data=False))
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
            self.pub.publish(Bool(data=True))
        else:
            self.pub.publish(Bool(data=False))


if __name__ == '__main__':
    rospy.init_node('people_pose_aggregated_detector')
    cagg = PeoplePoseAggregatedDetector()  # noqa
    rospy.spin()
