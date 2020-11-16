#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# from jsk_recognition_msgs.msg import BoundingBoxArray
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestActionGoal








def callbackfunc(msg):
    if msg.transcript:
        recognized_str = msg.transcript[0]
        print("coded_str={}".format(recognized_str))
        # decoded_str = coded_str.decode('unicode-escape')


        speak_msg = SoundRequestActionGoal()
        speak_msg.goal.sound_request.volume = 1.0
        speak_msg.goal.sound_request.command = 1
        speak_msg.goal.sound_request.sound = -3
        speak_msg.header.stamp = rospy.Time.now()
        speak_msg.goal_id.stamp = rospy.Time.now()
        speak_msg.goal.sound_request.arg = "いま、しゃべったのは、"+recognized_str+"ですね"
        print("Fetch says {}".format(speak_msg.goal.sound_request.arg))
        rospy.loginfo(speak_msg)
        pub.publish(speak_msg)






if __name__ == '__main__':
    try:
        rospy.init_node('fetch_recognition_str', anonymous=False)
        pub = rospy.Publisher('/robotsound_jp/goal', SoundRequestActionGoal, queue_size=1)
        rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, callbackfunc)
        rospy.spin()
    except rospy.ROSInterruptException: pass
