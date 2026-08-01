#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class VoiceFrontFilter:
    def __init__(self):
        self.threshold = 45
        self.current_direction = 0
        self.dir_sub = rospy.Subscriber("/sound_direction", Int32, self.dir_callback)
        self.stt_sub = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.stt_callback)
        self.filtered_pub = rospy.Publisher("/filtered_speech", SpeechRecognitionCandidates, queue_size=10)

    def dir_callback(self, msg):
        self.current_direction = msg.data

    def stt_callback(self, msg):
        angle = self.current_direction
        print("angle=", angle)
        is_front = (180 - self.threshold <= angle <= 180) or (-180 <= angle <= -180 + self.threshold)
        print("is_front=", is_front)

        if is_front:
            rospy.loginfo(f"Front voice detected ({angle} deg): {msg.transcript[0]}")
            self.filtered_pub.publish(msg)
        else:
            rospy.loginfo(f"Ignored voice from side ({angle} deg)")

if __name__ == '__main__':
    rospy.init_node('voice_front_filter')
    node = VoiceFrontFilter()
    rospy.spin()
