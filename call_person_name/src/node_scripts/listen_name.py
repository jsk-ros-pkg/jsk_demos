#!/usr/bin/env python3
import rospy
import time
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.libsoundplay import SoundClient

class NameListenerNode:
    def __init__(self):
        rospy.init_node("name_listener_node")

        # SoundClient 初期化
        self.sound_client = SoundClient(sound_action='sound_play', sound_topic='sound_play')
        rospy.sleep(1)  # 音声ノードの準備待ち

        # /speech_to_text トピックの購読
        self.sub = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.callback)

        rospy.loginfo("名前を待っています...")

    def callback(self, data):
        if not data.transcript:
            rospy.logwarn("音声が認識されませんでした。")
            return

        name = data.transcript[0]
        rospy.loginfo(f"認識された名前: {name}")

        # 名前を繰り返す
        response = f"{name}さんですか？"
        rospy.loginfo(f"応答: {response}")
        self.sound_client.say(response)
        rospy.loginfo(f"I said {response}")        

if __name__ == "__main__":
    try:
        node = NameListenerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
