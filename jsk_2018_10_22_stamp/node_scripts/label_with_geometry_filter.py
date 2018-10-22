#!/usr/bin/env python

import rospy
import message_filters

from jsk_recognition_msgs.msg import LabelArray
from jsk_recognition_msgs.msg import PoseArray
from jsk_topic_tools import ConnectionBasedTransport

from jsk_2018_10_22_stamp.msg import LabelWithGeometry


class LabelWithGeometryFilter(ConnectionBasedTransport):

    def __init__(self):
        self.pub = self.advertise(
            '~output',
            LabelWithGeometryFilter, queue_size=1)

    def subscribe(self):
        sub_label_array = message_filters.Subscriber(
            '~input/labels', LabelArray)
        sub_pose_array = message_filters.Subscriber(
            '~input/poses', PoseArray)
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 100)
        self.subs = [sub_label_array, sub_pose_array]
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, label_array_msg, pose_array_msg):
        msg = LabelWithGeometry()
        msg.header = label_array_msg.header
        msg.labelarray = label_array_msg
        msg.posearray = pose_array_msg
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('label_with_geometry_filter')
    app = LabelWithGeometryFilter()
    rospy.spin()
