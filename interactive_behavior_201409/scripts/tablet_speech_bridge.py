#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_gui_msgs.msg import VoiceMessage
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class TabletSpeechBridge(ConnectionBasedTransport):
    def __init__(self):
        super(TabletSpeechBridge, self).__init__()
        self.pub_speech = self.advertise(
            "speech_to_text", SpeechRecognitionCandidates,
            queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            "/Tablet/voice", VoiceMessage, self.callback)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, in_msg):
        out_msg = SpeechRecognitionCandidates()
        out_msg.transcript = in_msg.texts
        out_msg.confidence = [0.0] * len(in_msg.texts)
        if out_msg.confidence:
            out_msg.confidence[0] = 1.0
        self.pub_speech.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("tablet_speech_bridge")
    b = TabletSpeechBridge()
    rospy.spin()
