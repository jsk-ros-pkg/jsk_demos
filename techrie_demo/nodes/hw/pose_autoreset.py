#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, threading
from std_msgs.msg import String

class AutoReset:
    def __init__(self):
        rospy.init_node("pose_autoreset")
        self.delay = float(rospy.get_param("~delay_sec", 2.0))
        self.pub = rospy.Publisher("/motion/play", String, queue_size=10)
        rospy.Subscriber("/motion/play", String, self.cb, queue_size=50)
        rospy.loginfo("pose_autoreset: delay=%.2fs", self.delay)

    def cb(self, s:String):
        cmd = (s.data or "").strip()
        if not cmd: return
        if cmd == "pose:reset": return
        if cmd.startswith("pose:") or cmd.startswith("seq:"):
            threading.Timer(self.delay, lambda: self.pub.publish(String("pose:reset"))).start()

if __name__ == "__main__":
    AutoReset(); rospy.spin()
