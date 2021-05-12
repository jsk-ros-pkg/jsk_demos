#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
rospy.init_node('check', anonymous=True)

def callback(data):
    print(data.height, data.width)
    
rospy.Subscriber("/edgetpu_panorama_human_pose_estimator/output/image", Image, callback)

rospy.spin()
