#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy

def cb(m: Joy):
    pressed = [i for i,v in enumerate(m.buttons) if v]
    if pressed:
        rospy.loginfo("pressed buttons: %s  (len=%d)", pressed, len(m.buttons))

if __name__ == "__main__":
    rospy.init_node("joy_probe")
    rospy.Subscriber("/joy", Joy, cb, queue_size=50)
    rospy.loginfo("joy_probe: press some buttons…")
    rospy.spin()
