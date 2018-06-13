#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import speech_recognition as SR
from audio_common_msgs.msg import AudioData
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechToText(object):
    def __init__(self):

        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.sample_width = rospy.get_param("~sample_width", 2L)
        self.language = rospy.get_param("~language", "ja-JP")

        self.recognizer = SR.Recognizer()
        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = rospy.Subscriber("audio", AudioData, self.audio_cb)

    def audio_cb(self, msg):
        data = SR.AudioData(msg.data, self.sample_rate, self.sample_width)
        try:
            rospy.loginfo("Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_google(
                data, language=self.language)
            msg = SpeechRecognitionCandidates(transcript=[result])
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))


if __name__ == '__main__':
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()
