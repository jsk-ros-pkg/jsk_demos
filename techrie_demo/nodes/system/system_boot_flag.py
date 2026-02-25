#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("system_boot_flag")
    pub = rospy.Publisher("/system/started", Bool, queue_size=1, latch=True)
    rospy.sleep(0.05)
    pub.publish(Bool(False))
    rospy.loginfo("system_boot_flag: /system/started=False (latched)")
    rospy.spin()
