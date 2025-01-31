#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ros_speak import play_sound
from pathlib import Path

# 動作を実行する関数
def perform_action(neck_motion_message, eyebrow_status_message, sound_path):
    # 首と眉の動きを同時に実行
    neck_motion_pub.publish(neck_motion_message)
    eyebrow_status_pub.publish(eyebrow_status_message)
    rospy.loginfo(f"Published '{neck_motion_message}' to /neck_motion")
    rospy.loginfo(f"Published '{eyebrow_status_message}' to /eyebrow_status")

    # サウンド再生
    try:
        play_sound(sound_path, topic_name='robotsound_jp', wait=True)
        rospy.loginfo(f"Playing sound: {sound_path}")
    except Exception as e:
        rospy.logerr(f"Failed to play sound: {e}")

# コールバック関数
def action_callback(msg):
    if msg.data == "ask_balloon":
        perform_action("disagree", "angry", 'package://kxr_controller/resources/ask_balloon.wav')

def main():
    rospy.init_node('action_listener')

    # パブリッシャーの設定
    global neck_motion_pub, eyebrow_status_pub
    neck_motion_pub = rospy.Publisher('/neck_motion', String, queue_size=1)
    eyebrow_status_pub = rospy.Publisher('/eyebrow_status', String, queue_size=1)

    # サブスクライバーの設定
    rospy.Subscriber('/action_trigger', String, action_callback)

    rospy.loginfo("Action listener node started, waiting for messages...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
