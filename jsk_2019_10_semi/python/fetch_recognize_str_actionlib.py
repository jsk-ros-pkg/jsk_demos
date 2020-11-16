#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal

import actionlib


global client



def callbackfunc(msg):
    if msg.transcript:
        recognized_str = msg.transcript[0]
        print("coded_str={}".format(recognized_str))


        speak_msg = SoundRequestGoal()
        speak_msg.sound_request.volume = 1.0
        speak_msg.sound_request.command = 1
        speak_msg.sound_request.sound = -3
        speak_msg.sound_request.arg = "いま、しゃべったのは、"+recognized_str+"、ですね"
        speak_msg.sound_request.arg2 = 'ja'
        print("Fetch says {}".format(speak_msg.sound_request.arg))
        rospy.loginfo(speak_msg)
        client.send_goal(speak_msg)






if __name__ == '__main__':
    try:
        rospy.init_node('fetch_recognition_str', anonymous=False)
        # pub = rospy.Publisher('/robotsound_jp/goal', SoundRequestActionGoal, queue_size=1)
        rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, callbackfunc)
        client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        client.wait_for_server()
        rospy.spin()
    except rospy.ROSInterruptException: pass
