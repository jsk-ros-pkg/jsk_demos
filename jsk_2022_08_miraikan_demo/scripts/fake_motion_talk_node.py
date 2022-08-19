#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
import time

class Talk(object):
    def __init__(self):
        print("Start fake node of choregraph")
    
    def episode_fake(self):
        for i in range(16):
            print("fake node: {}".format(i))
            time.sleep(1)

class TopicToMotion(object):
    def __init__(self):
        self.talk_class = Talk()
        rospy.Subscriber("~input", Int32, self.motion_cb)
    
    def motion_cb(self, msg):
        self.talk_class.episode_fake()

if __name__ == '__main__':
    rospy.init_node("topic_to_motion")
    topic_to_motion = TopicToMotion()
    rospy.spin()
