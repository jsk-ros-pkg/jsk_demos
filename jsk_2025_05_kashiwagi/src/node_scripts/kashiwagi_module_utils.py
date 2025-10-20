#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, UInt16, ColorRGBA, Float32

class Modules:
    def __init__(self):
        # Eye publishers（左右で変数分け）
        self.eye_pub_left = rospy.Publisher("/left/eye_display/eye_status", String, queue_size=10)
        self.eye_pub_right = rospy.Publisher("/right/eye_display/eye_status", String, queue_size=10)

        # Cheek LED publishers
        self.cheek_led_blink_pub = rospy.Publisher("/cheek/led_blink_time", UInt16, queue_size=1)
        self.cheek_led_duration = rospy.Publisher("/cheek/led_duration", Float32, queue_size=1)
        self.cheek_led_mode = rospy.Publisher("/cheek/led_mode", UInt16, queue_size=1)
        self.cheek_led_rainbow_delta_hue = rospy.Publisher("/cheek/led_rainbow_delta_hue", UInt16, queue_size=1)
        self.cheek_led_rgb = rospy.Publisher("/cheek/led_rgb", ColorRGBA, queue_size=1)

        # Ume LED publishers
        self.ume_led_blink_pub = rospy.Publisher("/ume/led_blink_time", UInt16, queue_size=1)
        self.ume_led_duration = rospy.Publisher("/ume/led_duration", Float32, queue_size=1)
        self.ume_led_mode = rospy.Publisher("/ume/led_mode", UInt16, queue_size=1)
        self.ume_led_rainbow_delta_hue = rospy.Publisher("/ume/led_rainbow_delta_hue", UInt16, queue_size=1)
        self.ume_led_rgb = rospy.Publisher("/ume/led_rgb", ColorRGBA, queue_size=1)

    def eye(self, mode="normal"):
        msg = String(data=mode)
        self.eye_pub_left.publish(msg)
        self.eye_pub_right.publish(msg)

    def cheek_led(self, r, g, b, brightness=10, mode=1, blink=3, duration=1, rainbow_hue=1):
        self._publish_led(
            r, g, b, brightness, mode, blink, duration, rainbow_hue,
            self.cheek_led_blink_pub,
            self.cheek_led_duration,
            self.cheek_led_mode,
            self.cheek_led_rainbow_delta_hue,
            self.cheek_led_rgb
        )

    def ume_led(self, r, g, b, brightness=20, mode=1, blink=3, duration=1, rainbow_hue=1):
        self._publish_led(
            r, g, b, brightness, mode, blink, duration, rainbow_hue,
            self.ume_led_blink_pub,
            self.ume_led_duration,
            self.ume_led_mode,
            self.ume_led_rainbow_delta_hue,
            self.ume_led_rgb
        )

    def _publish_led(self, r, g, b, brightness, mode, blink, duration, rainbow_hue,
                     blink_pub, duration_pub, mode_pub, hue_pub, rgb_pub):
        color = ColorRGBA(r=r, g=g, b=b, a=brightness)
        blink_pub.publish(UInt16(blink))
        duration_pub.publish(Float32(duration))
        mode_pub.publish(UInt16(mode))
        hue_pub.publish(UInt16(rainbow_hue))
        rgb_pub.publish(color)
