#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

def main():
    rospy.init_node("diary_switch")
    btn = rospy.get_param("~button_index", 6)
    cli = rospy.ServiceProxy("/diary/make_today", Trigger)
    cli.wait_for_service(rospy.Duration(5.0))
    last = 0
    def cb(m: Joy):
        nonlocal last
        cur = int(m.buttons[btn]) if len(m.buttons) > btn else 0
        if cur and not last:
            try:
                res = cli()
                rospy.loginfo("diary: %s", res.message)
            except Exception as e:
                rospy.logwarn("diary_make_today failed: %s", e)
        last = cur
    rospy.Subscriber("/joy", Joy, cb, queue_size=20)
    rospy.loginfo("diary_switch ready (button=%d)", btn)
    rospy.spin()

if __name__ == "__main__":
    main()
