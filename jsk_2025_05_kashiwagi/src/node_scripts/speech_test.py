#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

def callback(msg):
    print("Speech to Text:", msg.transcript[0])

def main():
    rospy.init_node("speech_to_text_test")
    rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
