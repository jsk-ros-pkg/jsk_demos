#!/usr/bin/env python3
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

class Listener:
    def __init__(self):
        rospy.init_node("kashiwagi_listener")

        self.cur_kashiwagi_state = "unknown"

        self.sub_speech = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_callback)
        self.sub_speech = rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        rospy.loginfo("Launching listener node ....")
        rospy.spin()

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def speech_callback(self, msg):
        rospy.loginfo(msg.transcript[0])
        spoken_word = msg.transcript[0]
        req_state = None
        if self.cur_kashiwagi_state == "idle" and spoken_word == "おはよう":
            req_state = "daily:waking_up"
        elif self.cur_kashiwagi_state == "daily_normal" and spoken_word == "柏木さん":
            req_state = "daily:happy"
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word == "遊" or spoken_word =="遊ぼ" or spoken_word =="遊ぼう"):
            req_state  = "talking_game:listening_turn"
        elif (self.cur_kashiwagi_state == "talking_game:listening_turn" or self.cur_kashiwagi_state == "talking_game:speaking_turn") and (spoken_word == "おわり" or spoken_word == "終わり"):
            req_state = "daily:normal"

        if req_state != None:
            try:
                resp = self.set_state_srv(req_state)
                if resp.success:
                    rospy.loginfo(f"State updated: {resp.message}")
                else:
                    rospy.logwarn(f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        Listener()
    except rospy.ROSInterruptException:
        pass
