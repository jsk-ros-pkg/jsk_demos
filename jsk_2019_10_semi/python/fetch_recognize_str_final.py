#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal
import actionlib
from std_msgs.msg import Int16
#
# 赤　お菓子
# 青　GPU
# 緑　ぬいぐるみ

str_list = [["お菓子", "甘いもの", "腹減った", "あまいもの", "おかし", "はらへった"], ["計算資源", "計算遅い", "けいさん", "計算", "高速", "GPU"], ["やわらかい", "ぬいぐるみ", "もふもふ", "ソフト", "モフモフ"]]
present_list = {"red": 1, "blue": 2, "green": 3}

class speak_class:
    volume = 1.0
    command = 1
    sound = -3
    arg2 = 'ja'
    recognized_str = ""
    decided_str = ""
    pub = None
    flag_of_decision = False

    def __init__(self):
        rospy.init_node('fetch_recognition_str', anonymous=False)
        rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.callbackfunc)
        self.actionlib_part = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.actionlib_part.wait_for_server()
        rospy.Timer(rospy.Duration(10), self.loopOnce)
        self.pub = rospy.Publisher('/atohayoroshiku', Int16, queue_size = 1)

    def callbackfunc(self, msg):
        if msg.transcript:
            self.recognized_str = msg.transcript[0]
            print("coded_str={}".format(self.recognized_str))

    def loopOnce(self, event):
        speak_msg = SoundRequestGoal()
        speak_msg.sound_request.volume = self.volume
        speak_msg.sound_request.command = self.command
        speak_msg.sound_request.sound = self.sound
        speak_msg.sound_request.arg = "さっき、しゃべったのは、"+self.recognized_str+"、でしたね"
        speak_msg.sound_request.arg2 = self.arg2
        print("Fetch says {}".format(speak_msg.sound_request.arg))
        rospy.loginfo(speak_msg)
        #self.actionlib_part.send_goal(speak_msg)

        atohayoroshiku_msg = Int16()
        if self.flag_of_decision == False:
            if self.recognized_str in str_list[0]:
                self.flag_of_decision = True
                self.decided_str = self.recognized_str
                print("list of red : sweet")
                atohayoroshiku_msg.data = 1
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)
            elif self.recognized_str in str_list[1]:
                self.flag_of_decision = True
                self.decided_str = self.recognized_str
                print("list of blue : gpu")
                atohayoroshiku_msg.data = 2
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)
            elif self.recognized_str in str_list[2]:
                self.flag_of_decision = True
                self.decided_str = self.recognized_str
                print("list of green : nuigurumi")
                atohayoroshiku_msg.data = 3
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)
        else:
            if self.decided_str in str_list[0]:
                print("list of red : sweet")
                atohayoroshiku_msg.data = 1
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)
            elif self.decided_str in str_list[1]:
                print("list of blue : gpu")
                atohayoroshiku_msg.data = 2
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)
            elif self.decided_str in str_list[2]:
                print("list of green : nuigurumi")
                atohayoroshiku_msg.data = 3
                rospy.loginfo(atohayoroshiku_msg)
                self.pub.publish(atohayoroshiku_msg)










if __name__ == '__main__':
    try:
        obj = speak_class()
        rospy.spin()
    except rospy.ROSInterruptException: pass
