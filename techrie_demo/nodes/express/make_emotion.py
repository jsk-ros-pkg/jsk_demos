#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
import threading
from enum import Enum
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import ColorRGBA

class emotion_node:
    
    def __init__(self):
        self.led_blink_pub = rospy.Publisher("/led_blink_time", UInt16, queue_size=1)
        self.led_duration = rospy.Publisher("/led_duration", UInt16, queue_size=1)
        self.led_mode = rospy.Publisher("/led_mode", UInt16, queue_size=1)
        self.led_rainbow_delta_hue = rospy.Publisher("/led_rainbow_delta_hue", UInt16, queue_size=1)
        self.led_rgb = rospy.Publisher("/led_rgb", ColorRGBA, queue_size=1)
        
        
    def led(self,r,g,b,brightness=10,mode=1,blink=3,duration=1,rainbow_hue=1):
        
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = brightness
        
        blink_msg = UInt16()
        duration_msg = UInt16()
        mode_msg = UInt16()
        rainbow_hue_msg = UInt16()
        
        blink_msg.data = blink
        duration_msg.data = duration
        mode_msg.data = mode
        rainbow_hue_msg.data = rainbow_hue

        
        self.led_blink_pub.publish(blink_msg)
        self.led_duration.publish(duration_msg)
        self.led_mode.publish(mode_msg)
        self.led_rainbow_delta_hue.publish(rainbow_hue_msg)
        self.led_rgb.publish(color)
        
    def color_change(self,emotion):
        self.led(255,255,255,brightness=10,mode=1)
        if emotion == "joy":
            self.led(255,165,0) #orange
        elif emotion == "interest":
            self.led(255,105,180) #pink
        elif emotion == "anger":
            self.led(255,0,0) #red
        elif emotion == "bore":
            self.led(147,112,219) #purple
        elif emotion == "sad":
            self.led(65,105,255) #light blue
        elif emotion == "surprise":
            self.led(255,255,0) #yellow
        elif emotion == "fear":
            self.led(0,0,139) #blue
        elif emotion == "trust":
            self.led(60,179,113)
        elif emotion == "neutral":
            self.led(255,255,255)
