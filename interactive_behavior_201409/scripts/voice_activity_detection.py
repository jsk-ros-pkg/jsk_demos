#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import webrtcvad
import rospy
from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData


class VoiceActivityDetector(object):
    def __init__(self):
        self.audio_rate = rospy.get_param("~rate", 16000)
        self.audio_bit = rospy.get_param("~bitrate", 16)
        if self.audio_bit < 16:
            self.bitwidth = 1
        elif self.audio_bit == 16:
            self.bitwidth = 2
        else:
            self.bitwidth = 4
        self.min_duration = rospy.get_param("~min_duration", 0.1)
        self.max_duration = rospy.get_param("~max_duration", 7.0)
        self.tolerance = rospy.get_param("~tolerance", 0.3)
        self.publish_bool = rospy.get_param("~publish_bool", True)

        self.audio_buffer = str()
        self.vad = webrtcvad.Vad(3)
        self.is_speech = False
        self.speech_stopped = rospy.Time(0)

        self.pub_audio = rospy.Publisher("speech_audio", AudioData, queue_size=1)
        if self.publish_bool:
            self.pub_bool = rospy.Publisher("is_speeching", Bool, queue_size=1)
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        self.sub = rospy.Subscriber("audio", AudioData, self.callback)

    def callback(self, msg):
        now = rospy.Time.now()
        if self.vad.is_speech(msg.data, self.audio_rate):
            self.speech_stopped = now
        if now - self.speech_stopped < rospy.Duration(self.tolerance):
            self.is_speech = True
            self.audio_buffer += msg.data
        else:
            if self.is_speech:
                duration = 8.0 * len(self.audio_buffer) * self.bitwidth
                duration = duration / self.audio_rate / self.audio_bit
                rospy.loginfo("Duraiton: %f" % duration)
                if self.min_duration <= duration < self.max_duration:
                    rospy.loginfo("Published")
                    self.pub_audio.publish(AudioData(data=self.audio_buffer))
                self.audio_buffer = str()
                self.is_speech = False

    def timer_cb(self, event=None):
        self.pub_bool.publish(Bool(data=self.is_speech))


if __name__ == '__main__':
    rospy.init_node("voice_activity_detector")
    vad = VoiceActivityDetector()
    rospy.spin()
