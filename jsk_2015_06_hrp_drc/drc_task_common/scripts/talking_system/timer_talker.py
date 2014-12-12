#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
import time
import datetime
import locale

if __name__ == "__main__":
    rospy.init_node("timer_talker")
    talk_command_pub = rospy.Publisher('/robotsound', SoundRequest)
    time.sleep(10)
    talk_command_pub.publish(SoundRequest(sound=SoundRequest.PLAY_FILE, command=SoundRequest.PLAY_ONCE,
                                          arg=('http://translate.google.com/translate_tts?tl=en&q=%s' % 'Timer\ starting.')))
    while not rospy.is_shutdown():
        d = datetime.datetime.today()
        if d.minute == 0:
            talk_command_pub.publish(SoundRequest(sound=SoundRequest.PLAY_FILE, command=SoundRequest.PLAY_ONCE,
                                                  arg=('http://translate.google.com/translate_tts?tl=en&q=%s' % 'It\'s %s o\'clock\ now.' % d.hour)))
        time.sleep(30)

