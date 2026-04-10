#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse


class PhotoRecorder:
    def __init__(self):
        rospy.init_node("photo_recorder")
        self.bridge = CvBridge()
        self.latest = None

        image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.save_root = os.path.expanduser(
            rospy.get_param("~save_root", "~/.ros/techrie_demo/object_images")
        )
        os.makedirs(self.save_root, exist_ok=True)

        self.sub = rospy.Subscriber(image_topic, Image, self.cb, queue_size=1)
        self.srv = rospy.Service("/diary/snapshot", Trigger, self.handle)

        rospy.loginfo("photo_recorder: ready (topic=%s)", self.sub.resolved_name)
        rospy.loginfo("photo_recorder: save_root=%s", self.save_root)

    def cb(self, msg: Image):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("photo_recorder: cv_bridge failed: %s", e)

    def handle(self, _req):
        if self.latest is None:
            return TriggerResponse(False, "no image yet")

        now = datetime.now()
        day_dir = os.path.join(
            self.save_root,
            now.strftime("%Y"),
            now.strftime("%m"),
            now.strftime("%d"),
        )
        os.makedirs(day_dir, exist_ok=True)

        fname = now.strftime("%H%M%S") + ".jpg"
        path = os.path.join(day_dir, fname)

        try:
            ok = cv2.imwrite(path, self.latest)
            if not ok:
                return TriggerResponse(False, "save failed: cv2.imwrite returned False")
        except Exception as e:
            return TriggerResponse(False, "save failed: %s" % e)

        rospy.loginfo("photo_recorder: saved %s", path)
        return TriggerResponse(True, fname)


if __name__ == "__main__":
    PhotoRecorder()
    rospy.spin()
