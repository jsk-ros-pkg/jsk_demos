#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
import os

def talk_command_cb(msg):
    os.system("mplayer \'http://translate.google.com/translate_tts?tl=en&q=\'"+msg.arg+" > /dev/null 2>&1;")

if __name__ == "__main__":
    rospy.init_node("google_translation_talk")
    rospy.Subscriber('google_translation_talk_command', SoundRequest, talk_command_cb, queue_size=1)
    os.system("mplayer \'http://translate.google.com/translate_tts?tl=en&q=\'Talking\ System\ is\ starting. > /dev/null 2>&1;")
    #os.system("mplayer \'http://translate.google.com/translate_tts?tl=en&q=\'Talking\ System\ with\ Google\ Translation\ is\ launched. > /dev/null 2>&1;")
    rospy.spin()

