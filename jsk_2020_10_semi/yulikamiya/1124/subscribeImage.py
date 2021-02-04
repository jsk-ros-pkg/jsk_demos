#! /usr/bin/env python
import rospy
from std_msgs.msg import Image

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/camera/depth/image_rect_raw")
