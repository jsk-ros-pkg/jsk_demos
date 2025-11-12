#!/usr/bin/env python3
import rospy
import actionlib
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
from sound_play.libsoundplay import SoundClient

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
        self.greeting_wav_file = None
        rospy.loginfo("Launching listener node ....")
        rospy.spin()

    def _done_cb(self, state, result):
        rospy.loginfo("Speech finished!")
        self.is_speaking = False
        if self.cur_kashiwagi_state != "daily:happy":
            return
        else:
            try:
                resp = self.set_state_srv("daily:normal")
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def _feedback_cb(self, state):
        if self.is_self_introduction:
            req_state = "daily:introduction"
    
    def play_wav(self, file_path):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.PLAY_FILE  # ファイル再生モード
        goal.sound_request.command = SoundRequest.PLAY_ONCE  # 一回だけ再生
        goal.sound_request.arg = file_path  # 再生する wav ファイルのパス
        goal.sound_request.volume = 1.0     # 音量（0.0〜1.0）

        rospy.loginfo(f"Playing: {file_path}")
        self.client.send_goal(goal,
                              done_cb=self._done_cb,
                              feedback_cb=self._feedback_cb)
        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()
        rospy.loginfo("Playback finished.")
        
    def say_text(self, text):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = text
        # goal.sound_request.arg2 = "ちび式じい-ノーマル"
        goal.sound_request.arg2 = "ずんだもん-ノーマル"
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

    def is_mentioned(self, spoken_word, word_list):
        if any(word in spoken_word for word in word_list):
            return True
        else:
            return False

    def speech_callback(self, msg):
        rospy.loginfo(msg.transcript[0])
        spoken_word = msg.transcript[0]
        print(spoken_word)
        req_state = None
        if self.cur_kashiwagi_state == "idle" and self.is_mentioned(spoken_word, ["おはよう", "起きて", "おきて", "掟", "起き", "柏木さん", "柏", "柏木", "押上", "押上さん"]):
            req_state = "daily:waking_up"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["自己紹介", "自己", "事故"]):
            req_state = "daily:introduction"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_self_introduction.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["さようなら", "さよなら", "またね", "また"]):
            req_state = "daily:goodbye"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_matane.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木", "押上", "押上さん"]):
            req_state = "daily:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["ありがとう", "ありがと"]):
            req_state = "talking_game:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_arigato.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            req_state = "daily:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["おはよう"]):
            req_state = "daily:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["遊", "遊ぼ", "遊ぼう"]):
            req_state  = "talking_game:starting"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_iiyo.wav"
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木"]):
            req_state = "talking_game:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            req_state = "talking_game:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
        elif self.cur_kashiwagi_state in ["talking_game:listening_turn", "talking_game:speaking_turn"] and self.is_mentioned(spoken_word, ["おわり", "終わり", "ありがとう"]):
            req_state = "daily:happy"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_tanoshikattane.wav"
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["おやすみ", "おやす"]):
            req_state = "idle"
            self.greeting_wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_oyasumi.wav"
        else:
            self.greeting_wav_file = None

        if self.greeting_wav_file != None:
            print("playing")
            print(self.greeting_wav_file)
            self.play_wav(self.greeting_wav_file)

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
