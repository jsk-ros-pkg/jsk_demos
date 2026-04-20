#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from sensor_msgs.msg import Image
import cv2
import sys
import os
from cv_bridge import CvBridge
import subprocess

class VideoSaver(object):
    def __init__(self):
        self.video_path = rospy.get_param("~video_path", "")
        fps = rospy.get_param("~fps", 10)
        width = rospy.get_param("~width", None)
        height = rospy.get_param("~height", None)
        if not self.video_path:
            rospy.logfatal("no video path found")
            sys.exit(1)
        rospy.loginfo("video path: %s" % self.video_path)
        try:
            os.makedirs(os.path.dirname(self.video_path))
        except:
            pass
        self.cv_bridge = CvBridge()

        self.video = None
        if fps is not None and width is not None and height is not None:
            self.video = self.create_video(width, height, fps)
        self.stamps = []
        self.img_sub = rospy.Subscriber("image", Image, self.callback)

    def create_video(self, width, height, fps):
        codec = cv2.cv.CV_FOURCC(*"XVID")
        fn, _ = os.path.splitext(self.video_path)
        return cv2.VideoWriter(fn + ".avi", codec, fps, (width, height), True)

    def estimate(self, msg):
        if len(self.stamps) < 10:
            self.stamps.append(msg.header.stamp)
        else:
            stamp_size = len(self.stamps)
            dt = 0.0
            for i in range(stamp_size-1)[1:]:
                dt += (self.stamps[i+1]-self.stamps[i]).to_sec()
            dt /= stamp_size
            rospy.loginfo("fps estimated: %f" % (1.0 / dt))
            self.video = self.create_video(msg.width, msg.height, 1.0 / dt)

    def callback(self, msg):
        if self.video is None:
            self.estimate(msg)
        else:
            mat = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.video.write(mat)
    def on_shutdown(self):
        if self.video.isOpened():
            self.video.release()
        fn, _ = os.path.splitext(self.video_path)
        cmd = "avconv -i %s.avi -c:v libx264 -c:a copy %s.mp4" % (fn, fn)
        subprocess.Popen(cmd, shell=True)

if __name__ == '__main__':
    rospy.init_node("video_saver")
    v = VideoSaver()
    rospy.on_shutdown(v.on_shutdown)
    rospy.spin()
