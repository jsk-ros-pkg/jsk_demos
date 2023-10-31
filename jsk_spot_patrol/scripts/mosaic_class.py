#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray
from cv_bridge import CvBridge


class mosaic:
    def __init__(self, ratio=0.1):
        self.rects_list = []
        self.ratio = ratio
        self.sub_rects = rospy.Subscriber(
            "/spot_recognition/rects",
            RectArray,
            self.cut_area)
        self.sub_image = rospy.Subscriber(
            "/spot_recognition/object_detection_image",
            Image,
            self.mosaic_area_around)

    def cut_area(self, msg):
        self.rects_list = msg.rects

    def mosaic_area_around(self, msg):
        try:
            max_width = msg.width
            max_height = msg.height
            bridge = CvBridge()
            orig = bridge.imgmsg_to_cv2(msg, "bgr8")
            dst = orig.copy()
            if self.rects_list:
                for rect in rects_list:
                    
            # cv2.imshow('image_area', dst)
            # cv2.waitKey(1)
        except Exception as err:
            print err

def main():
    rospy.init_node('mosaic')
    rospy.loginfo('mosaic node started')

if __name__ == '__main__':
    main()
