#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String, Float32
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
import time

class KashiwagiSpeaker:
    def __init__(self):
        self.current_kashiwagi_state = "unknown"
        self.prev_kashiwagi_state = "unknown"
        rospy.init_node('kashiwagi_speaker')
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        rospy.loginfo("Launching kashiwagi speaker node ....")
        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()
        self.is_thinking = False
        rospy.spin()

    def play_wav(self, file_path):
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.PLAY_FILE  # ファイル再生モード
        goal.sound_request.command = SoundRequest.PLAY_ONCE  # 一回だけ再生
        goal.sound_request.arg = file_path  # 再生する wav ファイルのパス
        goal.sound_request.volume = 1.0     # 音量（0.0〜1.0）

        rospy.loginfo(f"Playing: {file_path}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Playback finished.")

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data
        if self.current_kashiwagi_state == "talking_game:thinking_turn":
            if self.is_thinking == False:
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_hmm.wav"
                self.play_wav(wav_file)
                self.is_thinking = True
        else:
            self.is_thinking = False

if __name__ == '__main__':
    try:
        KashiwagiSpeaker()
    except rospy.ROSInterruptException:
        pass
