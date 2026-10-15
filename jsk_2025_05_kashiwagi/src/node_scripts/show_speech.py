#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print recognised speech as text you can read.

rostopic echo renders the message as YAML, and its emitter escapes anything
outside ASCII, so Japanese arrives as \\u30C6\\u30B9\\u30C8 rather than テスト.
Subscribing directly avoids the round trip through YAML entirely.

    rosrun jsk_2025_05_kashiwagi show_speech.py
    rosrun jsk_2025_05_kashiwagi show_speech.py _interim:=true
"""
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String


def main():
    rospy.init_node("show_speech")
    show_interim = rospy.get_param("~interim", False)

    def on_final(msg):
        for i, text in enumerate(msg.transcript):
            conf = msg.confidence[i] if i < len(msg.confidence) else float("nan")
            rospy.loginfo("%s  (%.2f)", text.strip(), conf)

    def on_interim(msg):
        rospy.loginfo("... %s", msg.data.strip())

    rospy.Subscriber(rospy.get_param("~topic", "/speech_to_text"),
                     SpeechRecognitionCandidates, on_final, queue_size=10)
    if show_interim:
        rospy.Subscriber("/speech_to_text/interim", String, on_interim,
                         queue_size=10)
    rospy.loginfo("listening on %s", rospy.get_param("~topic", "/speech_to_text"))
    rospy.spin()


if __name__ == "__main__":
    main()
