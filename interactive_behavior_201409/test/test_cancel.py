#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_gui_msgs.msg import VoiceMessage


pub = None


def pub_speech(text):
    global pub
    msg = VoiceMessage(texts=[text])
    if pub is None:
        pub = rospy.Publisher(
            "/Tablet/voice", VoiceMessage, queue_size=1)
        rospy.sleep(1)
    pub.publish(msg)


def test_cancel():
    pub_speech("ねえねえ")
    rospy.sleep(3)
    pub_speech("テストってツイートしてください")
    rospy.sleep(3)
    pub_speech("ストップ")


if __name__ == '__main__':
    rospy.init_node("test_cancel")
    test_cancel()
