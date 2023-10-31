#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray
from cv_bridge import CvBridge

def mosaic(msg, ratio=0.1):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        small = cv2.resize(
            orig, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        )
        large = cv2.resize(
            small, orig.shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        )
        #img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('image', large)
        # cv2.waitKey(1)
    except Exception as err:
        print err

def mosaic_area(msg, x=50, y=50, width=500, height=400, ratio=0.1):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        dst = orig.copy()
        #print dst[0:20,0:20]
        #dst = np.array(dst)
        #dst[y:y + height, x:x + width] = mosaic(dst[y:y + height, x:x + width], ratio)
        #dst[0:20, 0:20] = mosaic(dst[0:20], ratio)
        #img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(
            dst[y:y+height, x:x+width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        )
        large = cv2.resize(
            small, dst[y:y+height, x:x+width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        )
        dst[y:y+height, x:x+width] = large
        cv2.imshow('image', large)
        cv2.imshow('image_area', dst)
        cv2.waitKey(1)
    except Exception as err:
        print err

def mosaic_area_around(msg):
    try:
        max_width = msg.width
        max_height = msg.height
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        dst = orig.copy()
        #print dst[0:20,0:20]
        #dst = np.array(dst)
        #dst[y:y + height, x:x + width] = mosaic(dst[y:y + height, x:x + width], ratio)
        #dst[0:20, 0:20] = mosaic(dst[0:20], ratio)
        #img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        if rects_list:
            for l in rects_list:
                small = cv2.resize(
                    dst[0:max_height, 0:l.x], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
                )
                large = cv2.resize(
                    small, dst[0:max_height, 0:l.x].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
                )
                dst[0:max_height, 0:l.x] = large
                
                small = cv2.resize(
                    dst[0:max_height, l.x+l.width:max_width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
                )
                large = cv2.resize(
                    small, dst[0:max_height, l.x+l.width:max_width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        )
                dst[0:max_height, l.x+l.width:max_width] = large
        
                small = cv2.resize(
                    dst[0:l.y, l.x:l.x+width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
                )
                large = cv2.resize(
                    small, dst[0:l.y, l.x:l.x+l.width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
                )
                dst[0:l.y, l.x:l.x+l.width] = large
                cv2.imshow('image', large)
                small = cv2.resize(
                    dst[l.y+l.height:max_height, l.x:l.x+l.width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
                )
                large = cv2.resize(
                    small, dst[l.y+l.height:max_height, l.x:l.x+l.width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
                )
                dst[l.y+l.height:max_height, l.x:l.x+l.width] = large
        else:
            print "none of rects"
        # small = cv2.resize(
        #     dst[0:max_height, 0:x], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        # )
        # large = cv2.resize(
        #     small, dst[0:max_height, 0:x].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        # )
        # dst[0:max_height, 0:x] = large
        
        # small = cv2.resize(
        #     dst[0:max_height, x+width:max_width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        # )
        # large = cv2.resize(
        #     small, dst[0:max_height, x+width:max_width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        # )
        # dst[0:max_height, x+width:max_width] = large
        
        # small = cv2.resize(
        #     dst[0:y, x:x+width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        # )
        # large = cv2.resize(
        #     small, dst[0:y, x:x+width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        # )
        # dst[0:y, x:x+width] = large
        # cv2.imshow('image', large)
        # small = cv2.resize(
        #     dst[y+height:max_height, x:x+width], None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST
        # )
        # large = cv2.resize(
        #     small, dst[y+height:max_height, x:x+width].shape[:2][::-1], interpolation=cv2.INTER_NEAREST
        # )
        # dst[y+height:max_height, x:x+width] = large
        # cv2.imshow('image', large)
        cv2.imshow('image_area', dst)
        cv2.waitKey(1)
    except Exception as err:
        print err

def cut_area (msg):
    global rects_list
    rects_list = msg.rects
    # if rects_list:
    #     print rects_list
    # else:
    #     print "none of rects"

def start_node():
    global x, y, width, height,ratio, rects_list
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    #rospy.Subscriber("/kinova_wrist_camera/color/image_raw", Image, mosaic)
    #rospy.Subscriber("/kinova_wrist_camera/color/image_raw", Image, mosaic_area_around)
    rospy.Subscriber("/spot_recognition/rects", RectArray, cut_area)
    rospy.Subscriber("/spot_recognition/object_detection_image", Image, mosaic_area_around)
    # x = 50
    # y = 50
    # width = 400
    # height = 400
    x = rospy.get_param("~target_x", 50)
    y = rospy.get_param("~target_y", 50)
    width = rospy.get_param("~width", 500)
    height = rospy.get_param("~height", 300)
    ratio = 0.1
    print "hoge"
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
