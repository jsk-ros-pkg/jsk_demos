#!/usr/bin/env python3
import rospy
import os
import rospkg
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

class ProposeGameSpeechTrigger:
    def __init__(self):
        rospy.init_node("propose_game_speech_trigger")
        self.cur_state = "unknown"
        self.path_to_pkg = rospkg.RosPack().get_path("jsk_2025_05_kashiwagi")
        self.tmp_response_path = os.path.join(self.path_to_pkg, "data", "tmp", "tmp_response.txt")
        self.pub = rospy.Publisher("/talking_game_response", String, queue_size=1)
        self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)
        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_callback, queue_size=1)
        rospy.loginfo("Propose Game Speech Trigger started")
        rospy.spin()

    def state_callback(self, msg):
        self.cur_state = msg.data

    def is_mentioned(self, spoken_word, word_list):
        return any(word in spoken_word for word in word_list)

    def speech_callback(self, msg):
        if self.cur_state != "propose_game:ready_to_speak" or not msg.transcript:
            return
        spoken_word = msg.transcript[0]
        rospy.loginfo(f"recognized: {spoken_word}")
        if not self.is_mentioned(spoken_word, ["柏木さんの番", "かしわぎさんの番", "柏木の番", "柏木さんの晩"]):
            return
        rospy.loginfo("Propose game speaking trigger detected")
        try:
            resp = self.set_state_srv("propose_game:preparing_speech")
            if not resp.success:
                rospy.logwarn(f"State update failed: {resp.message}")
                return
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return
        try:
            with open(self.tmp_response_path, "r", encoding="utf-8") as f:
                proposal_text = f.read().strip()
        except Exception as e:
            rospy.logerr(f"Failed to read proposal text: {e}")
            return
        rospy.sleep(0.1)
        self.pub.publish(proposal_text)
        rospy.loginfo(f"published proposal: {proposal_text}")

if __name__ == "__main__":
    try:
        ProposeGameSpeechTrigger()
    except rospy.ROSInterruptException:
        pass
