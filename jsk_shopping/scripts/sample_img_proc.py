#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import cv2
import numpy as np

def process_image(msg):
    try:
        bridge = CvBridge()
        #orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        orig = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        img = img[100:380, 140:500]
        threshold = 100
        ret, img_bnr = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
        avg = cv2.mean(img_bnr)
        print avg[0]
        cv2.rectangle(orig, (140, 100), (500, 380), (0, 0, 255))
        
        # winner between black and white
        pub_bw = rospy.Publisher('black_or_white', Int16, queue_size=10)
        if avg[0]>190:
            pub_bw.publish(1)
        if avg[0]<=190:
            pub_bw.publish(0)

        #imgMsg = bridge.cv2_to_imgmsg(img, "area")
        #pub_img = rospy.Publisher('img_rect', Image, queue_size=10)
        #pub_img.publish(imgMsg)
        
        cv2.imshow('image',img_bnr)
        cv2.imshow('image_raw', orig)
        cv2.waitKey(1)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    
    rospy.Subscriber("/head_camera/rgb/image_rect_color/compressed",CompressedImage,process_image)
    #rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
