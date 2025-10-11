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
        self.wav_pub = rospy.Publisher('/robotsound_jp', SoundRequest, queue_size=10)
        self.is_thinking = False
        rospy.spin()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data
        if self.current_kashiwagi_state == "talking_game:thinking_turn":
            if self.is_thinking == False:
                wav_msg = SoundRequest()
                wav_msg.sound   = SoundRequest.PLAY_FILE
                wav_msg.command = SoundRequest.PLAY_ONCE
                wav_msg.volume  = 1.0
                wav_msg.arg = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/kashiwagi_hmm.wav"
                wav_msg.arg2    = ""
                self.wav_pub.publish(wav_msg)
                self.is_thinking = True
        else:
            self.is_thinking = False

if __name__ == '__main__':
    try:
        KashiwagiSpeaker()
    except rospy.ROSInterruptException:
        pass
