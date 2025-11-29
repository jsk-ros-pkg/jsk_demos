#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
import time

class KashiwagiSpeaker:
    def __init__(self):
        rospy.init_node('kashiwagi_speaker')
        self.cur_state = "unknown"
        self.prev_state = "unknown"
        self.last_play_time = 0.0  # 最後に再生した時間
        self.repeat_interval = 3.0 # move:happy時の繰り返し秒数
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
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
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_hmm.wav"
            elif self.cur_state == "katakanashi:thinking_turn":
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_hmm.wav"
            elif self.cur_state == "move:getting_lost":
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_megamawaru.wav"
            elif self.cur_state == "move:goal":
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_kitayo.wav"
            elif self.cur_state == "move:approaching_person":
                wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yoisho.wav"
                self.last_play_time = time.time()

            if wav_file:
                self.play_wav(wav_file)

    def timer_callback(self, event):
        if self.cur_state != "move:approaching_person":
            return

        now = time.time()
        if now - self.last_play_time >= self.repeat_interval:
            wav_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_yoisho.wav"
            self.play_wav(wav_file)
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
