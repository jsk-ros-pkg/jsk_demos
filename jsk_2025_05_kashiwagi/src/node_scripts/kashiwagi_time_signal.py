#!/usr/bin/env python3
import rospy
import sys, os, rospkg
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


class TimeSignal:
    def __init__(self):
        rospy.init_node("kashiwagi_time_signal")
        rospy.sleep(1.0)
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)

        self.cur_kashiwagi_state = "unknown"
        self.sub_state = rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.schedule_and_soundfile = {
            "15:00":f"{self.path_to_pkg}/data/kashiwagi_time_signal_1500.wav",
            "15:50":f"{self.path_to_pkg}/data/kashiwagi_time_signal_1550.wav",
            "16:00":f"{self.path_to_pkg}/data/kashiwagi_time_signal_1600.wav"}
        self.client.wait_for_server()
        self.sound_file = None
        self.after_speech_req_state = None
        self.during_speech_req_state = None
        self.last_fired_key = None
        self.is_updated = False
        rospy.loginfo("Launching listener node ....")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.on_timer)
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

    def on_timer(self, event):
        now = datetime.now()
        hhmm = now.strftime("%H:%M")
        fired_key = now.strftime("%Y-%m-%d %H:%M")

        if hhmm in self.schedule_and_soundfile.keys() and fired_key != self.last_fired_key:
            self.last_fired_key = fired_key
            rospy.loginfo("Time signal: %s", fired_key)
            self.during_speech_req_state = "daily:time_signal"
            self.after_speech_req_state = None
            self.play_sound_file(self.schedule_and_soundfile[hhmm])

        else:
            self.during_speech_req_state = None
            self.after_speech_req_state = None
            self.sound_file = None

        if self.sound_file != None:
            print("playing")
            print(self.sound_file)
            self.play_sound_file(self.sound_file)

if __name__ == "__main__":
    TimeSignal()
    rospy.spin()
