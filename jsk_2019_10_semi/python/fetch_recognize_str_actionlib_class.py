#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal
import actionlib

class speak_class:
    volume = 1.0
    command = 1
    sound = -3
    arg2 = 'ja'

    def __init__(self):
        rospy.init_node('fetch_recognition_str', anonymous=False)
        rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.callbackfunc)
        self.actionlib_part = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.actionlib_part.wait_for_server()

    def callbackfunc(self, msg):
        if msg.transcript:
            recognized_str = msg.transcript[0]
            print("coded_str={}".format(recognized_str))

            speak_msg = SoundRequestGoal()
            speak_msg.sound_request.volume = self.volume
            speak_msg.sound_request.command = self.command
            speak_msg.sound_request.sound = self.sound
            speak_msg.sound_request.arg = "いま、しゃべったのは、"+recognized_str+"、ですね"
            speak_msg.sound_request.arg2 = self.arg2
            print("Fetch says {}".format(speak_msg.sound_request.arg))
            rospy.loginfo(speak_msg)
            self.actionlib_part.send_goal(speak_msg)


    # 
    # def loop(self):
    #     rospy.spin()







if __name__ == '__main__':
    try:
        obj = speak_class()
        rospy.spin()
    except rospy.ROSInterruptException: pass
