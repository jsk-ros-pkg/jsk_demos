#!/usr/bin/env python
PKG = 'elevator_move_base_pr2'
import roslib; roslib.load_manifest(PKG)

import sys, unittest, time, rospy
from std_msgs.msg import Float32

TEST_DURATION = 3

## A sample python unit test
class TestColorPointDetector(unittest.TestCase):
    def callback(self, msg):
        print("msg.data = ", msg.data, " should be 1.5")
        self.assert_( abs( msg.data - 1.5 ) <  0.1  )
 
    def test_light_button(self):
        rospy.init_node(PKG, anonymous=True)
        sub = rospy.Subscriber("/light_button", Float32, self.callback)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and time.time() < timeout_t:
            time.sleep(0.1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_data', TestColorPointDetector)
