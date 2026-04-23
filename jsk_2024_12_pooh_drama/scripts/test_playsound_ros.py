#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# from __future__ import division

# # from dotenv import load_dotenv
# # load_dotenv()
# import os

# import shutil
# import array
# import asyncio
# from base64 import b64encode
# from io import BytesIO
# from threading import Lock
# import traceback
# import time
# from datetime import datetime
# import websockets

# from audio_common_msgs.msg import AudioData
# import numpy as np
# from pydub import AudioSegment
# from pydub import effects
# import rospy
# import soundfile as sf

# from audio_buffer import AudioBuffer
#from std_msgs.msg import UInt16
import rospy
from std_msgs.msg import String
from ros_speak import play_sound
from pathlib import Path




# 音声ファイルのパス（ROSパッケージパスを使用）
sound_file_path = 'package://kxr_controller/resources/ask_balloon.wav'

def play_audio(msg):
    rospy.loginfo("[play_audio]: msg.data={}".format(msg.data))
    if msg.data == "ask_balloon":
        try:
            # 音声ファイルを再生
            play_sound(sound_file_path, topic_name='robotsound_jp', wait=True)
            rospy.loginfo("Playing sound: ask_balloon.wav")
        except Exception as e:
            rospy.logerr(f"Failed to play sound: {e}")

def main():
    rospy.init_node('audio_player')
    
    # サブスクライバーの設定
    rospy.Subscriber('/action_trigger', String, play_audio)
    
    rospy.loginfo("Audio player node started, waiting for messages...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
