#!/usr/bin/env python3
import rospy
import actionlib
import time
import subprocess
import os
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class ResponseSpeakerWithAction:
    def __init__(self):
        rospy.init_node('response_speaker_action_node')
        rospy.sleep(1)
        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        rospy.loginfo("Waiting for sound_play action server...")
        self.client.wait_for_server()
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        self.is_speaking = False
        rospy.loginfo("Connected to sound_play action server.")
        # rospy.Subscriber("/talking_game_response", String, self.say_text)
        self.text_path = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/tmp/tmp_response.txt"
        self.wav_file_path = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/tmp/tmp_response.wav"
        rospy.Subscriber("/talking_game_response", String, self.generate_wav_and_play_sound_file)
        rospy.spin()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data

    def _done_cb(self, state, result):
        rospy.loginfo("Speech finished!")
        self.is_speaking = False
        try:
            if self.current_kashiwagi_state.split(":")[0] == "talking_game":
                resp = self.set_state_srv("talking_game:listening_turn")
            elif self.current_kashiwagi_state.split(":")[0] == "katakanashi":
                resp = self.set_state_srv("katakanashi:playing")
            elif self.current_kashiwagi_state.split(":")[0] == "shiritori":
                resp = self.set_state_srv("shiritori:listening_turn")
            rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _feedback_cb(self, state):
        if (self.is_speaking == False):
            try:
                if self.current_kashiwagi_state.split(":")[0] == "talking_game":
                    resp = self.set_state_srv("talking_game:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "katakanashi":
                    resp = self.set_state_srv("katakanashi:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "shiritori":
                    resp = self.set_state_srv("shiritori:speaking_turn")
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        self.is_speaking = True

    def generate_wav_and_play_sound_file(self, msg):
        cmd = ["rosrun", "voicevox", "text2wave", "-o", self.wav_file_path, self.text_path, "-eval", "(3)"]
        rospy.loginfo("Running VoiceVox text2wave...")
        print("aaaaaaaaaaaaaaaaaaaaaaaaa")
        result = subprocess.run(cmd, capture_output=True, text=True)
        print("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
        
        if result.returncode != 0:
            rospy.logerr(f"VoiceVox error:\n{result.stderr}")
            return
        else:
            rospy.loginfo("VoiceVox processing complete!")

        if os.path.exists(self.wav_file_path):
            goal = SoundRequestGoal()
            goal.sound_request.sound = SoundRequest.PLAY_FILE  # ファイル再生モード
            goal.sound_request.command = SoundRequest.PLAY_ONCE  # 一回だけ再生
            goal.sound_request.arg = self.wav_file_path  # 再生する wav ファイルのパス
            goal.sound_request.volume = 1.0     # 音量（0.0〜1.0）
            self.client.send_goal(goal,
                                  done_cb=self._done_cb,
                                  feedback_cb=self._feedback_cb)
            self.client.wait_for_result()
            rospy.loginfo("Playback finished.")
        else:
            rospy.logerr("WAV file not found!")
        
    def say_text(self, msg):
        text = msg.data.replace("\n", "")
        rospy.loginfo(f"Talking contents: {text}")

        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = text
        goal.sound_request.arg2 = "ずんだもん-ノーマル"
        goal.sound_request.volume = 1.0

        # 音声再生を送信
        self.client.send_goal(goal,
                              done_cb=self._done_cb,
                              feedback_cb=self._feedback_cb)
        
        # 表情アニメーションを喋ってる間だけ繰り返す（wait_for_result中）
        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()

if __name__ == "__main__":
    try:
        ResponseSpeakerWithAction()
    except rospy.ROSInterruptException:
        pass
