#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
rospy.init_node('check', anonymous=True)

def callback(data):
    print(data.height, data.width)
    
rospy.Subscriber("/dual_fisheye_to_panorama/output", Image, callback)

rospy.spin()
