#!/usr/bin/env python3
import sys, os, rospkg
import rospy
import actionlib
from std_msgs.msg import String
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
import time
import random

class KashiwagiSpeaker:
    def __init__(self):
        rospy.init_node('kashiwagi_speaker')
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)

        self.cur_state = "unknown"
        self.prev_state = "unknown"
        self.last_play_time = 0.0  # 最後に再生した時間
        self.repeat_interval = 3.0 # move:happy時の繰り返し秒数
        self.thinking_wav_files = [f"{self.path_to_pkg}/data/kashiwagi_thinking.wav",
                                   f"{self.path_to_pkg}/data/kashiwagi_thinking_2.wav",
                                   f"{self.path_to_pkg}/data/kashiwagi_thinking_3.wav"]
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback, queue_size=1)
        rospy.loginfo("Launching kashiwagi speaker node ....")

        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()

        # 定期チェック用タイマー
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        rospy.spin()

    def state_callback(self, msg):
        self.prev_state = self.cur_state
        self.cur_state = msg.data

        self.state_updated = (self.prev_state != self.cur_state)

        wav_file = None

        if self.state_updated:
            if self.cur_state == "talking_game:thinking_turn":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_hmm.wav"
                self.last_play_time = time.time()
            elif self.cur_state == "katakanashi:thinking_turn":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_hmm.wav"
                self.last_play_time = time.time()
            elif self.cur_state == "shiritori:thinking_turn":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_hmm.wav"
                self.last_play_time = time.time()
            elif self.cur_state == "move:getting_lost":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_megamawaru.wav"
            elif self.cur_state == "free_talk:thinking_turn":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_thinking_2.wav"
            elif self.cur_state == "move:goal":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_kitayo.wav"
            elif self.cur_state == "move:approaching_person":
                wav_file = f"{self.path_to_pkg}/data/kashiwagi_yoisho.wav"
                self.last_play_time = time.time()

            if wav_file:
                self.play_wav(wav_file)

    def timer_callback(self, event):
        thinking_wav_file = random.choice(self.thinking_wav_files)
        repeat_config = {
            "move:approaching_person": {
                "wav": f"{self.path_to_pkg}/data/kashiwagi_yoisho.wav",
                "interval": 3.0,
            },
            "talking_game:thinking_turn": {
                "wav": thinking_wav_file,
                "interval": 5.0,
            },
            "katakanashi:thinking_turn": {
                "wav": thinking_wav_file,
                "interval": 5.0,
            },
            "shiritori:thinking_turn": {
                "wav": thinking_wav_file,
                "interval": 8.0,
            },
        }

        cfg = repeat_config.get(self.cur_state)
        if cfg is None:
            # 繰り返し再生対象外の state
            return

        now = time.time()

        if now - self.last_play_time >= cfg["interval"]:
            self.play_wav(cfg["wav"])
            self.last_play_time = now

    def play_wav(self, file_path):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.PLAY_FILE
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = file_path
        goal.sound_request.volume = 1.0

        rospy.loginfo(f"Playing: {file_path}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Playback finished.")

if __name__ == '__main__':
    try:
        KashiwagiSpeaker()
    except rospy.ROSInterruptException:
        pass
