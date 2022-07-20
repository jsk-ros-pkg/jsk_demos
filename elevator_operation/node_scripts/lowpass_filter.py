#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


class LowpassFilterNode(object):

    #
    # See https://qiita.com/yuji0001/items/b0bf121fb8b912c02856
    #

    def __init__(self):

        self.filter_order = rospy.get_param('~filter_order', 3)
        self.freq_cutoff = rospy.get_param('~freq_cutoff', 0.1)
        self.buffer_length = rospy.get_param('~buffer_length', 100)
        self.buffer = list()
        self.buffer_stamp = list()
        self.pre_time = None
        self.pre_acc_z = None
        self.initialized = False
        self.pub = rospy.Publisher('~output', Float32, queue_size=1)
        self.sub = rospy.Subscriber('~input', Float32, self.callback)

    def callback(self, msg):

        current_stamp = rospy.Time.now()
        if self.pre_acc_z is None or self.pre_time is None:
            self.pre_acc_z = msg.data
            self.pre_time = current_stamp
        else:
            dt = (current_stamp - self.pre_time).to_sec()
            fs = 1.0 / dt
            rospy.loginfo('fs: {}'.format(fs))
            self.pre_acc_z = (1 - (self.freq_cutoff / fs)) * self.pre_acc_z \
                + (self.freq_cutoff / fs) * msg.data
            self.pre_time = current_stamp

        self.pub.publish(Float32(data=self.pre_acc_z))


if __name__ == '__main__':

    rospy.init_node('lowpass_filter')
    node = LowpassFilterNode()
    rospy.spin()
