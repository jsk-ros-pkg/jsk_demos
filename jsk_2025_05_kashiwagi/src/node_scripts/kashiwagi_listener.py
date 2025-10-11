#!/usr/bin/env python3
import rospy
import actionlib
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest

class Listener:
    def __init__(self):
        rospy.init_node("kashiwagi_listener")
        rospy.sleep(1.0)

        self.cur_kashiwagi_state = "unknown"

        self.sub_speech = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_callback)
        self.sub_speech = rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()

        rospy.loginfo("Launching listener node ....")
        rospy.spin()

    def say_text(self, text):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = text
        goal.sound_request.arg2 = "ちび式じい-ノーマル"
        goal.sound_request.volume = 1.0

        # 音声再生を送信
        self.client.send_goal(goal)
        
        # 表情アニメーションを喋ってる間だけ繰り返す（wait_for_result中）
        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()

        rospy.loginfo("Speech finished!")
        
    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def speech_callback(self, msg):
        rospy.loginfo(msg.transcript[0])
        spoken_word = msg.transcript[0]
        req_state = None
        if self.cur_kashiwagi_state == "idle" and (spoken_word in ["おはよう", "起きて", "おきて", "掟"]):
            req_state = "daily:waking_up"
            self.say_text("おはよう")
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word in ["さようなら", "またね"]):
            ### TODO ###
            self.say_text("またねー")
            req_state = "daily:goodbye"
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word in ["柏木さん", "柏", "柏木"]):
            self.say_text("呼んだ？")
            req_state = "daily:happy"
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word in ["こんにちは"]):
            self.say_text("こんにちは")
            req_state = "daily:happy"
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word in ["遊", "遊ぼ", "遊ぼう"]):
            ### TODO ### 
            req_state  = "talking_game:starting"
            self.say_text("いいよー")

        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and (spoken_word in ["柏木さん", "柏", "柏木"]):
            self.say_text("呼んだ？")
            req_state = "talking_game:happy"
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and (spoken_word in ["こんにちは"]):
            self.say_text("こんにちは")
            req_state = "talking_game:happy"

        elif self.cur_kashiwagi_state in ["talking_game:listening_turn", "talking_game:speaking_turn"] and (spoken_word in ["おわり", "終わり"]):
            req_state = "daily:happy"
            self.say_text("楽しかったね")
        elif self.cur_kashiwagi_state == "daily:normal" and (spoken_word == "おやすみ"):
            self.say_text("おやすみ")
            req_state = "idle"

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
