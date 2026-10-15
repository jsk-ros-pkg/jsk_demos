#!/usr/bin/env python3
import rospy
import sys, os, rospkg
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
        self.current_kashiwagi_state = "unknown"
        rospy.Subscriber('/kashiwagi_state', String, self.state_callback, queue_size=1)
        self.is_speaking = False
        rospy.loginfo("Connected to sound_play action server.")
        # rospy.Subscriber("/talking_game_response", String, self.say_text)
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)
        self.text_path = f"{self.path_to_pkg}/data/tmp/tmp_response.txt"
        self.wav_file_path = f"{self.path_to_pkg}/data/tmp/tmp_response.wav"
        rospy.Subscriber("/talking_game_response", String, self.generate_wav_and_play_sound_file, queue_size=1)
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
            elif self.current_kashiwagi_state.split(":")[0] == "free_talk":
                resp = self.set_state_srv("free_talk:listening_turn")
            elif self.current_kashiwagi_state.split(":")[0] == "propose_game":
                resp = self.set_state_srv("propose_game:happy")
            else:
                rospy.logwarn(f"Unknown Kashiwagi state after speech: {self.current_kashiwagi_state}")
                return
            rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _feedback_cb(self, state):
        if self.is_speaking == False:
            try:
                if self.current_kashiwagi_state.split(":")[0] == "talking_game":
                    resp = self.set_state_srv("talking_game:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "katakanashi":
                    resp = self.set_state_srv("katakanashi:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "shiritori":
                    resp = self.set_state_srv("shiritori:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "free_talk":
                    resp = self.set_state_srv("free_talk:speaking_turn")
                elif self.current_kashiwagi_state.split(":")[0] == "propose_game":
                    resp = self.set_state_srv("propose_game:speaking_turn")
                else:
                    rospy.logwarn(f"Unknown Kashiwagi state while speaking: {self.current_kashiwagi_state}")
                    self.is_speaking = True
                    return
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        self.is_speaking = True

    def generate_wav_and_play_sound_file(self, msg):
        rospy.loginfo(f"speech trigger received: {msg.data}")
        cmd = ["rosrun", "voicevox", "text2wave", "-o", self.wav_file_path, self.text_path, "-eval", "(3)"]
        rospy.loginfo("Running VoiceVox text2wave...")
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            rospy.logerr(f"VoiceVox error:\n{result.stderr}")
            return
        rospy.loginfo("VoiceVox processing complete!")

        if os.path.exists(self.wav_file_path):
            goal = SoundRequestGoal()
            goal.sound_request.sound = SoundRequest.PLAY_FILE
            goal.sound_request.command = SoundRequest.PLAY_ONCE
            goal.sound_request.arg = self.wav_file_path
            goal.sound_request.volume = 1.0
            self.client.send_goal(goal, done_cb=self._done_cb, feedback_cb=self._feedback_cb)
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
        self.client.send_goal(goal, done_cb=self._done_cb, feedback_cb=self._feedback_cb)
        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()

if __name__ == "__main__":
    try:
        ResponseSpeakerWithAction()
    except rospy.ROSInterruptException:
        pass
