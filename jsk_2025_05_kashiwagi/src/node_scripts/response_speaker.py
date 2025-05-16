#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class ResponseSpeaker:
    def __init__(self):
        rospy.init_node('response_speaker_node')
        self.sound_client = SoundClient(sound_action="/robotsound_jp", blocking=True)
        rospy.sleep(1)  # 初期化待ち
        rospy.Subscriber("/gpt_reply", String, self.say_text)
        rospy.loginfo("GPTSpeaker ノードが起動しました。/gpt_reply を購読中...")
        rospy.spin()

    def say_text(self, msg):
        text = msg.data
        text = text.replace("\n", "")
        rospy.loginfo(f"話す内容: {text}")
        self.sound_client.say(text, voice='ちび式じい-ノーマル')

if __name__ == "__main__":
    try:
        ResponseSpeaker()
    except rospy.ROSInterruptException:
        pass
