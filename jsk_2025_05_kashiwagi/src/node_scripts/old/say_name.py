#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import FaceArrayStamped
import time
import json
import os
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

DATA_FILE = "last_seen.json"
NICKNAME_FILE = "nicknames.json"

last_seen_times = {}
nicknames = {}
sound_client = None

# 呼び方（ニックネーム）ファイルを読み込む
def load_nicknames():
    global nicknames
    if os.path.exists(NICKNAME_FILE):
        with open(NICKNAME_FILE, 'r') as f:
            nicknames = json.load(f)
        rospy.loginfo("ニックネームファイルを読み込みました")
    else:
        rospy.logwarn("ニックネームファイルが見つかりませんでした。label をそのまま使用します。")

# 最後に人を見た時刻のデータをファイルから読み込む
def load_last_seen():
    global last_seen_times
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, 'r') as f:
            last_seen_times = json.load(f)
        rospy.loginfo("記録ファイルを読み込みました")
    else:
        rospy.loginfo("記録ファイルが見つかりませんでした（新規スタート）")

# 最後にその人を見た時刻のデータをファイルに保存する
def save_last_seen():
    with open(DATA_FILE, 'w') as f:
        json.dump(last_seen_times, f)

# コールバック関数
def callback(msg):
    global sound_client
    
    current_time = time.time()
    updated = False

    for face in msg.faces:
        label = face.label
        if not label:
            continue  # ラベルなしはスキップ
        
        name = nicknames.get(label, label)
        
        if label in last_seen_times:
            delta = current_time - float(last_seen_times[label])
            rospy.loginfo("再検出: %s（前回から %.2f 秒経過）", name, delta)
            if delta >= 10.0:
                message = f"{name}、おかえり"
                print(message)
                sound_client.say(message, voice='ja')
        else:
            rospy.loginfo("初見の顔: %s", name)
            sound_client.say(message, voice='ja')

        last_seen_times[label] = current_time
        updated = True

    if updated:
        save_last_seen()

def listener():
    global sound_client
    rospy.init_node('face_name_listener', anonymous=True)

    sound_client = SoundClient(sound_action="/robotsound_jp", blocking=True)
    rospy.sleep(1)
    
    load_last_seen()
    load_nicknames()

    rospy.Subscriber("face_name", FaceArrayStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
