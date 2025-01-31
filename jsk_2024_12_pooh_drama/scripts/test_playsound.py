#!/usr/bin/env python

import rospy
from playsound import playsound

def play_wav_file(file_path):
    """
    指定されたWAVファイルを再生します。

    Args:
        file_path (str): 再生するWAVファイルのパス。
    """
    try:
        print(f"再生中: {file_path}")
        playsound(file_path)
        print("再生完了")
    except Exception as e:
        print(f"エラーが発生しました: {e}")

if __name__ == "__main__":
    rospy.init_node('wav_player', anonymous=True)

    # 再生したいWAVファイルのパスを指定してください
    file_path = "/home/leus/ros/catkin_ws/src/rcb4/ros/kxr_controller/resources/ask_balloon.wav"
    
    # WAVファイルを再生
    play_wav_file(file_path)
