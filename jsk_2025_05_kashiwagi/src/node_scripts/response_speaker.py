#!/usr/bin/env python3
import rospy
import actionlib
import time
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
        self.is_speaking = False
        rospy.loginfo("Connected to sound_play action server.")
        rospy.Subscriber("/talking_game_response", String, self.say_text)
        rospy.spin()

    def _done_cb(self, state, result):
        rospy.loginfo("Speech finished!")
        self.is_speaking = False
        try:
            resp = self.set_state_srv("talking_game:listening_turn")
            rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def _feedback_cb(self, state):
        if (self.is_speaking == False):
            try:
                resp = self.set_state_srv("talking_game:speaking_turn")
                rospy.loginfo(f"State updated: {resp.message}" if resp.success else f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        self.is_speaking = True
    
    def say_text(self, msg):
        text = msg.data.replace("\n", "")
        rospy.loginfo(f"Talking contents: {text}")

        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = text
        goal.sound_request.arg2 = "ちび式じい-ノーマル"
        # goal.sound_request.arg2 = "ずんだもん-ノーマル"
        goal.sound_request.volume = 1.0

        # 音声再生を送信
        self.client.send_goal(goal,
                              done_cb=self._done_cb,
                              feedback_cb=self._feedback_cb)
        
        # 表情アニメーションを喋ってる間だけ繰り返す（wait_for_result中）
        rate = rospy.Rate(10)
        while not self.client.wait_for_result(timeout=rospy.Duration(0.1)):
            rate.sleep()

        # # rospy.loginfo("Speech finished!")
        # # change kashiwagi state to "talking_game:listening_turn"
        # try:
        #     req_state = "talking_game:listening_turn"
        #     resp = self.set_state_srv(req_state)
        #     if resp.success:
        #         rospy.loginfo(f"State updated: {resp.message}")
        #     else:
        #         rospy.logwarn(f"State update failed: {resp.message}")
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        ResponseSpeakerWithAction()
    except rospy.ROSInterruptException:
        pass
