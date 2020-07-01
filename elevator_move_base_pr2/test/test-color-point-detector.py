#!/usr/bin/env python

import time
import unittest

import rospy
import rostest
from std_msgs.msg import Float32


class TestColorPointDetector(unittest.TestCase):

    def setUp(self):
        self.timeout = rospy.get_param('~timeout', 3.0)
        self.target_value = rospy.get_param('~target_value', 1.5)
        self.msg = None
        self.sub = rospy.Subscriber('/light_button', Float32, self.callback)

    def callback(self, msg):
        self.msg = msg

    def test_light_button(self):
        timeout_t = time.time() + self.timeout
        while not rospy.is_shutdown() and time.time() < timeout_t:
            if self.msg is not None:
                rospy.loginfo(
                    'msg.data should be {}, actual: {}'.format(
                        self.target_value, self.msg.data))
                self.assertTrue(abs(self.msg.data - self.target_value) < 0.1)
                return
            time.sleep(0.1)
        self.fail('Test timeout.')


if __name__ == '__main__':
    rospy.init_node('test_color_point_detector')
    rostest.rosrun('elevator_move_base_pr2', 'test_color_point_detector',
                   TestColorPointDetector)
