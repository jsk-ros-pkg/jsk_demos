#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
from datetime import datetime
import os


class StateLogger(object):
    def __init__(self):
        rospy.init_node("kashiwagi_state_logger")

        # 保存先パス（パラメータで上書きも可）
        default_path = os.path.expanduser("/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/kashiwagi_state_log.json")
        self.output_path = rospy.get_param("~output_path", default_path)

        self.prev_state = None
        self.log = []  # 変更履歴をここに溜める

        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)

        rospy.loginfo("State logger started. Output: %s", self.output_path)
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data

        # 最初の一回は prev_state が None なので、そのままセットして終了
        if self.prev_state is None:
            self.prev_state = new_state
            self.append_and_save(new_state)
            return

        # 前回と同じなら何もしない
        if new_state == self.prev_state:
            return

        # 変更があったときだけ記録
        self.prev_state = new_state
        self.append_and_save(new_state)

    def append_and_save(self, state_str):
        # 現在時刻（人間が読める形式＆ISO8601）
        now = datetime.now().isoformat()

        # ROS 時刻（必要なら使えるように入れておく）
        ros_now = rospy.Time.now()

        entry = {
            "state": state_str,
            "timestamp": now,  # 人が読みやすい
            "ros_time": {
                "secs": ros_now.secs,
                "nsecs": ros_now.nsecs
            }
        }

        self.log.append(entry)

        # JSON として保存
        try:
            with open(self.output_path, "w") as f:
                json.dump(self.log, f, ensure_ascii=False, indent=2)
            rospy.loginfo("State updated: %s", entry)
        except IOError as e:
            rospy.logerr("Failed to write JSON file: %s", e)


if __name__ == "__main__":
    try:
        StateLogger()
    except rospy.ROSInterruptException:
        pass
