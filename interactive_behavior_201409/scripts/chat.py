#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
import os
import readline
import rospy
import sys
import threading

from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestActionGoal
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


HISTORY = os.path.expanduser("~/.speech_history")
PROMPT = "{talker}: "


def speech_cb(msg):
    if isinstance(msg, SoundRequestActionGoal):
        return speech_cb(msg.goal.sound_request)
    if msg.sound != SoundRequest.SAY:
        return
    print("\r\033[K", end="")
    print(PROMPT.format(talker="robot"), msg.arg)
    print(PROMPT.format(talker="you  "), readline.get_line_buffer(), sep="", end="")
    sys.stdout.flush()


def on_shutdown():
    readline.write_history_file(HISTORY)


def subscribe():
    subs = []
    topics = rospy.get_published_topics()
    t = filter(lambda t: t[1] == SoundRequest._type, topics)
    for name, type in t:
        sub = rospy.Subscriber(name, SoundRequest, speech_cb)
        subs.append(sub)
    t = filter(lambda t: t[1] == SoundRequestActionGoal._type, topics)
    for name, type in t:
        sub = rospy.Subscriber(name, SoundRequestActionGoal, speech_cb)
        subs.append(sub)
    return subs


def command():
    pub = rospy.Publisher("/speech_to_text", SpeechRecognitionCandidates, queue_size=1)
    msg = SpeechRecognitionCandidates()
    while not rospy.is_shutdown():
        text = raw_input(PROMPT.format(talker="you  "))
        if not text:
            print("You've logged out")
            rospy.signal_shutdown("shutdown")
        msg.transcript = [text]
        pub.publish(msg)


def main():
    readline.parse_and_bind("set editing-mode emacs")
    try:
        readline.read_history_file(HISTORY)
    except:
        pass

    rospy.init_node("chat")

    rospy.on_shutdown(on_shutdown)

    subs = subscribe()

    threading.Thread(target=command).start()

    rospy.loginfo("Welcome to chat!")
    rospy.spin()


if __name__ == '__main__':
    main()
