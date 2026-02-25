#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

MAP = {8:"red", 9:"blue", 10:"yellow", 11:"green", 12:"orange", 13:"purple",14:"pink",15:"white"}  # ボタン→色
def main():
    rospy.init_node("color_selector")
    pub = rospy.Publisher('/paint_color', String, queue_size=1)
    def cb(m: Joy):
        for idx, pressed in enumerate(m.buttons):
            if pressed and idx in MAP:
                pub.publish(String(MAP[idx]))
    rospy.Subscriber('/joy', Joy, cb)
    rospy.spin()
if __name__=='__main__':
    main()
