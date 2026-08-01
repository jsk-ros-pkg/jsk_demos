#!/usr/bin/env python3
import rospy
import sys, os, rospkg
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
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)

        self.sub_speech = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_callback)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        self.client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.client.wait_for_server()
        self.greeting_wav_file = None

        rospy.loginfo("Launching listener node ....")
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

    def speech_callback(self, msg):
        rospy.loginfo(msg.transcript[0])
        spoken_word = msg.transcript[0]
        print(spoken_word)
        if (spoken_word in ["おはよう", "起きて", "おきて", "掟"]):
            self.greeting_wav_file = f"{self.path_to_pkg}/data/kashiwagi_ohayou.wav"

        if self.greeting_wav_file != None:
            print("playing")
            self.play_wav(self.greeting_wav_file)

if __name__ == "__main__":
    try:
              Listener()
    except rospy.ROSInterruptException:
        pass
