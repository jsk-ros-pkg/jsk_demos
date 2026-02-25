#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
from kxr_controller.msg import ServoOnOff

def cb(m: Bool):
    if not m.data:
        
        rospy.logwarn("system_guard: received system stop → shutting down launch...")
        rospy.signal_shutdown("system stop requested")

def main():
    rospy.init_node("system_guard")
    rospy.Subscriber("/system/started", Bool, cb, queue_size=1)
    rospy.loginfo("system_guard ready (required). Waiting for /system/started=False to shutdown all.")
    rospy.spin()

if __name__ == "__main__":
    main()
