#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge
import numpy as np

def draw_rect_cb(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "32FC1")
        
        # orig = bridge.imgmsg_to_cv2(msg.data, "orig")
        # orig = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        #img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        img = orig[100:380, 140:500]
        #img = img / float(1024)
        scaled_img = cv2.convertScaleAbs(orig, alpha=float(256)/1024, beta=0)
        
        # threshold = 100
        #ret, img_bnr = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
        #avg = cv2.mean(orig)
        #print avg
        scaled_img = cv2.cvtColor(scaled_img, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(scaled_img, (140, 100), (500, 380), (0, 0, 255))

        #imgmsg = bridge.cv2_to_imgmsg(orig, "area")
        #pub_img = rospy.Publisher('img_rect', Image, queue_size=10)
        #pub_img.publish(imgmsg)

        #cv2.imshow('image', img_bnr)
        #print orig
        cv2.imshow('image_raw', scaled_img)
        cv2.waitKey(1)
        
    except Exception as err:
        print err

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')

    rospy.Subscriber("/head_camera/depth/image_rect_raw", Image, draw_rect_cb)
    # rospy.Subscriber("/head_camera/depth_downsample/image_raw/compressedDepth", CompressedImage, draw_rect_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
