#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import json
import os
import yaml
import rospkg
from sound_play.libsoundplay import SoundClient
from opencv_apps.msg import FaceArrayStamped
from apriltag_ros.msg import AprilTagDetectionArray

class NameCaller:
    def __init__(self):
        # パッケージパス取得
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('call_person_name')

        # ファイルパス
        self.data_file = os.path.join(self.pkg_path, 'src', 'config', 'last_seen.json')
        self.nickname_file = os.path.join(self.pkg_path, 'src', 'config', 'nicknames.json')
        self.tags_yaml_file = os.path.join(self.pkg_path, 'src', 'config', 'tags.yaml')

        # データ
        self.last_seen_times = {}
        self.nicknames = {}
        self.tag_id_to_name = {}

        # 初期化
        self.load_last_seen()
        self.load_nicknames()
        self.load_tag_names()

        self.sound_client = SoundClient(sound_action="/robotsound_jp", blocking=True)
        rospy.sleep(1)

        rospy.Subscriber("/face_name", FaceArrayStamped, self.face_callback)
        rospy.Subscriber("/camera/color/tag_detections", AprilTagDetectionArray, self.tag_callback)

    def load_last_seen(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, 'r') as f:
                self.last_seen_times = json.load(f)
            rospy.loginfo("Loaded last_seen.json")
        else:
            rospy.loginfo("Could not find last_seen.json => make new json file")

    def save_last_seen(self):
        with open(self.data_file, 'w') as f:
            json.dump(self.last_seen_times, f)

    def load_nicknames(self):
        if os.path.exists(self.nickname_file):
            with open(self.nickname_file, 'r') as f:
                self.nicknames = json.load(f)
            rospy.loginfo("Loaded nicknames.json")
        else:
            rospy.logwarn("Could not find nicknames.json")

    def load_tag_names(self):
        if os.path.exists(self.tags_yaml_file):
            with open(self.tags_yaml_file, 'r') as f:
                tags_data = yaml.safe_load(f)
                for tag in tags_data.get('standalone_tags', []):
                    tag_id = tag.get('id')
                    tag_name = tag.get('name')
                    if tag_id is not None and tag_name:
                        self.tag_id_to_name[tag_id] = tag_name
            rospy.loginfo("Loaded tags.yaml")
        else:
            rospy.logwarn("Could not find tags.yaml")

    def call_name(self, label):
        if label not in self.nicknames:
            # rospy.loginfo(f"{label} is not registered in nicknames.json")
            return
        
        current_time = time.time()
        name = self.nicknames.get(label, label)
        message = ""
        updated = False

        if label in self.last_seen_times:
            delta = current_time - float(self.last_seen_times[label])
            if delta >= 10.0:
                message = f"{name}、おかえり"
                updated = True
        else:
            message = f"{name}、こんにちは"
            updated = True

        if updated:
            rospy.loginfo(message)
            self.sound_client.say(message, voice='ja')
            self.last_seen_times[label] = current_time
            self.save_last_seen()

    def face_callback(self, msg):
        for face in msg.faces:
            if face.label:
                self.call_name(face.label)

    def tag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            tag_name = self.tag_id_to_name.get(tag_id, f"tag_{tag_id}")
            self.call_name(tag_name)

if __name__ == '__main__':
    rospy.init_node('name_call_node')
    name_caller = NameCaller()
    rospy.spin()
