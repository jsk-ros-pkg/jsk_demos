#!/usr/bin/env python3
import rospy
import actionlib
import random
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
from sound_play.libsoundplay import SoundClient

# ここから追加 import --------------------
import json
from datetime import datetime
import os
# ここまで追加 --------------------------


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
        self.sound_file = None
        self.after_speech_req_state = None
        self.during_speech_req_state = None
        self.is_updated = False
        rospy.loginfo("Launching listener node ....")
        rospy.spin()

    def _done_cb(self, state, result):
        rospy.loginfo("Speech finished!")
        self.is_updated = False
        if self.after_speech_req_state != None:
            try:
                resp = self.set_state_srv(self.after_speech_req_state)
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def _feedback_cb(self, state):
        if self.is_updated == False and self.during_speech_req_state != None:
            try:
                resp = self.set_state_srv(self.during_speech_req_state)
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
                self.is_updated = True
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                
    
    def play_sound_file(self, file_path):
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

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def is_mentioned(self, spoken_word, word_list):
        if any(word in spoken_word for word in word_list):
            return True
        else:
            return False

    # ここから追加: state & spoken_word ログ関数 --------------------
    def log_state_and_speech(self, state_str, spoken_word):
        """
        現在の state と、そのときの spoken_word を JSON Lines 形式で保存する。
        デフォルト保存先: ~/kashiwagi_speech_state_log.jsonl
        パラメータ ~log_path で上書き可能。
        """
        log_path_default = os.path.expanduser("/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/kashiwagi_speech_state_log.jsonl")
        log_path = rospy.get_param("~log_path", log_path_default)

        entry = {
            "timestamp": datetime.now().isoformat(),
            "state": state_str,
            "spoken_word": spoken_word
        }

        try:
            # 1行1JSONで追記
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except IOError as e:
            rospy.logerr("Failed to write log file: %s", e)
    # ここまで追加 -------------------------------------------------

    def speech_callback(self, msg):
        rospy.loginfo(msg.transcript[0])
        spoken_word = msg.transcript[0]
        print(spoken_word)

        req_state = None
        if self.cur_kashiwagi_state == "idle" and self.is_mentioned(spoken_word, ["おはよう", "起きて", "おきて", "掟", "起き", "柏木さん", "柏", "柏木", "押上", "押上さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = "daily:waking_up"
            self.after_speech_req_state = None
            self.sound_file= "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["自己紹介", "自己", "事故"]):
            self.during_speech_req_state = "daily:introduction"
            self.after_speech_req_state = "daily:happy"
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_self_introduction_2.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["今日の予定"]):
            self.during_speech_req_state = "daily:introduction"
            self.after_speech_req_state = "daily:happy"
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yotei_12_10.wav"
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["さようなら", "さよなら", "またね", "また", "さよう", "バイバイ", "ばいばい"]):
            self.during_speech_req_state = "daily:goodbye"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_matane.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["ありがとう", "ありがと"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_arigato.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["かわいい", "可愛い"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ehehe.wav"
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["おはよう"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["遊", "遊ぼ", "遊ぼう"]):
            self.during_speech_req_state = "talking_game:starting"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_iiyo.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["カタカナ"]):
            self.during_speech_req_state = "katakanashi:starting"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_iiyo.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["しりとり"]):
            self.during_speech_req_state = "shiritori:starting"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_iiyo.wav"

            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["宜しく", "よろしく", "お願い"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yoroshiku.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["すごい", "人気"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_sorehododemo.wav"

#################
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            self.during_speech_req_state = "talking_game:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
            
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["かわいい", "可愛い", "すごい", "人気"]):
            self.during_speech_req_state = "talking_game:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ehehe.wav"

        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["おはよう"]):
            self.during_speech_req_state = "talking_game:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
            
        elif self.cur_kashiwagi_state in ["talking_game:listening_turn","talking_game:thinking_turn", "talking_game:speaking_turn"] and self.is_mentioned(spoken_word, ["おわり", "終わり", "ありがとう"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_tanoshikattane.wav"
############
        elif self.cur_kashiwagi_state == "katakanashi:playing" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            self.during_speech_req_state = "katakanashi:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
            
        elif self.cur_kashiwagi_state == "katakanashi:playing" and self.is_mentioned(spoken_word, ["かわいい", "可愛い"]):
            self.during_speech_req_state = "katakanashi:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ehehe.wav"

        elif self.cur_kashiwagi_state == "katakanashi:playing" and self.is_mentioned(spoken_word, ["おはよう"]):
            self.during_speech_req_state = "katakanashi:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
            
        elif self.cur_kashiwagi_state in ["katakanashi:playing", "katakanashi:thinking_turn", "katakanashi:speaking_turn"] and self.is_mentioned(spoken_word, ["おわり", "終わり"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_tanoshikattane.wav"

        elif self.cur_kashiwagi_state == "katakanashi:playing" and self.is_mentioned(spoken_word, ["ありがとう", "ありがと"]):
            self.during_speech_req_state = "katakanashi:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_arigato.wav"

###########
        elif self.cur_kashiwagi_state == "shiritori:listening_turn" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            self.during_speech_req_state = "shiritori:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
            
        elif self.cur_kashiwagi_state == "shiritori:listening_turn" and self.is_mentioned(spoken_word, ["かわいい", "可愛い"]):
            self.during_speech_req_state = "shiritori:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ehehe.wav"

        elif self.cur_kashiwagi_state == "shiritori:listening_turn" and self.is_mentioned(spoken_word, ["おはよう"]):
            self.during_speech_req_state = "shiritori:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"
            
        elif self.cur_kashiwagi_state in ["shiritori:listening_turn", "shiritori:thinking_turn", "shiritori:speaking_turn"] and self.is_mentioned(spoken_word, ["おわり", "終わり"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_tanoshikattane.wav"
            with open("/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/latest_shiritori_reply.txt", "w", encoding="utf-8") as f:
                f.write("")

        elif self.cur_kashiwagi_state == "shiritori:listening_turn" and self.is_mentioned(spoken_word, ["ありがとう", "ありがと"]):
            self.during_speech_req_state = "shiritori:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_arigato.wav"
###########            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["おやすみ", "おやす"]):
            self.during_speech_req_state = "idle"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_oyasumi.wav"

        elif self.cur_kashiwagi_state in ["daily:normal"] and self.is_mentioned(spoken_word, ["おいで", "こっち", "来て", "家に"]):
            self.during_speech_req_state = None
            self.after_speech_req_state = "move:finding_person"
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_darekayonda.wav"

        elif self.cur_kashiwagi_state == "move:approaching_person" and self.is_mentioned(spoken_word, ["危", "落ち", "止"]):
            self.during_speech_req_state = "move:surprised"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_bikkuri.wav"

        elif self.cur_kashiwagi_state in ["move:getting_lost", "move:staying"] and self.is_mentioned(spoken_word, ["おいで", "こっち", "来て", "家に", "柏木さん", "柏", "柏木", "押上", "押上さん", "西脇さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = None
            self.after_speech_req_state = "move:finding_person"
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_dokodaro.wav"

        elif self.cur_kashiwagi_state in ["move:getting_lost", "move:finding_person", "move:approaching_person", "move:staying"] and self.is_mentioned(spoken_word, ["ありがとう", "ありがと", "休んで", "終わり", "おわり"]):
            self.during_speech_req_state = None
            self.after_speech_req_state = "daily:happy"
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_tanoshikattane.wav"

        elif self.cur_kashiwagi_state == "move:staying" and self.is_mentioned(spoken_word, ["さようなら", "さよなら", "またね", "また", "さよう", "バイバイ", "ばいばい"]):
            self.during_speech_req_state = "daily:goodbye"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_matane.wav"

        elif self.cur_kashiwagi_state == "move:staying" and self.is_mentioned(spoken_word, ["ありがとう", "ありがと"]):
            self.during_speech_req_state = "move:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_arigato.wav"

        elif self.cur_kashiwagi_state == "move:staying" and self.is_mentioned(spoken_word, ["かわいい", "可愛い", "すごい", "人気"]):
            self.during_speech_req_state = "move:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ehehe.wav"
            
        elif self.cur_kashiwagi_state == "move:staying" and self.is_mentioned(spoken_word, ["こんにちは", "こんにち"]):
            self.during_speech_req_state = "move:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_konnichiwa.wav"
            
        elif self.cur_kashiwagi_state == "move:staying" and self.is_mentioned(spoken_word, ["おはよう"]):
            self.during_speech_req_state = "move:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_ohayou.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["どんぐり", "ドングリ", "ころころ", "コロコロ"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_donguri.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["雪やこんこん"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_yuki.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["でんでん"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_katatsumuri.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["桃太郎"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_momotaro.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["アイドル"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_idle.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["仰げば尊"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_aogeba.mp3"
            self.sound_file = song
            
        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["ぽっぽ"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            song = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_hato.mp3"
            self.sound_file = song

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["歌", "歌って"]):
            self.during_speech_req_state = "daily:singing"
            self.after_speech_req_state = "daily:happy"
            songs = ["/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_yuki.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_donguri.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_katatsumuri.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_momotaro.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_hato.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_aogeba.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_sakura.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_usagi.mp3",
                     "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_song_idle.mp3"
            ]
            self.sound_file = random.choice(songs)
        
        elif self.cur_kashiwagi_state == "talking_game:listening_turn" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木", "押上", "押上さん", "西脇さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = "talking_game:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"

        elif self.cur_kashiwagi_state == "katakanashi:playing" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木", "押上", "押上さん", "西脇さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = "katakanashi:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"

        elif self.cur_kashiwagi_state == "shiritori:listening_turn" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木", "押上", "押上さん", "西脇さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = "shiritori:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"

        elif self.cur_kashiwagi_state == "daily:normal" and self.is_mentioned(spoken_word, ["柏木さん", "柏", "柏木", "押上", "押上さん", "西脇さん", "西脇", "芦屋駅"]):
            self.during_speech_req_state = "daily:happy"
            self.after_speech_req_state = None
            self.sound_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yonda.wav"

        else:
            self.during_speech_req_state = None
            self.after_speech_req_state = None
            self.sound_file = None

        if req_state != None:
            try:
              resp = self.set_state_srv(req_state)
              if resp.success:
                rospy.loginfo(f"State updated: {resp.message}")
              else:
                rospy.logwarn(f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
              rospy.logerr(f"Service call failed: {e}")

        if self.sound_file != None:
            print("playing")
            print(self.sound_file)
            self.play_sound_file(self.sound_file)
            # 現在の self.cur_kashiwagi_state と spoken_word を保存
            self.log_state_and_speech(self.cur_kashiwagi_state, spoken_word)

if __name__ == "__main__":
    try:
        Listener()
    except rospy.ROSInterruptException:
        pass
