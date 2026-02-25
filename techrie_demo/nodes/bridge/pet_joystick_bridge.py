#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, math
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class PetJoystickBridge:
    def __init__(self):
        rospy.init_node("pet_joystick_bridge")
        self.th_on  = float(rospy.get_param("~threshold_on", 180))
        self.th_off = float(rospy.get_param("~threshold_off",155))
        self.alpha  = float(rospy.get_param("~ewma_alpha", 0.3))
        self.min_interval = float(rospy.get_param("~min_interval_sec", 1.2))

        self.pub = rospy.Publisher("/human/pet", String, queue_size=3)
        rospy.Subscriber("/joystick_xy", Vector3, self.cb, queue_size=30)

        self.ewma = 0.0
        self.active = False
        self.next_ok = rospy.Time(0)
        rospy.loginfo("pet_joystick_bridge: th_on=%.2f th_off=%.2f alpha=%.2f",
                      self.th_on, self.th_off, self.alpha)

    def cb(self, v:Vector3):
        mag = math.hypot(v.x, v.y)
        self.ewma = self.alpha*mag + (1.0-self.alpha)*self.ewma

        now = rospy.Time.now()
        if not self.active and self.ewma >= self.th_on and now >= self.next_ok:
            self.pub.publish(String("joystick"))
            self.active = True
            self.next_ok = now + rospy.Duration(self.min_interval)
        elif self.active and self.ewma <= self.th_off:
            self.active = False

if __name__ == "__main__":
    PetJoystickBridge(); rospy.spin()
