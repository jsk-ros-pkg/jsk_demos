#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os, cv2, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from rospkg import RosPack
from datetime import datetime

class PhotoRecorder:
    def __init__(self):
        rospy.init_node("photo_recorder")
        self.bridge = CvBridge()
        self.latest = None
        self.sub = rospy.Subscriber(rospy.get_param("~image_topic","/camera/color/image_raw"),
                                    Image, self.cb, queue_size=1)
        self.srv = rospy.Service("/diary/snapshot", Trigger, self.handle)
        self.pkg_root = RosPack().get_path("techrie_demo")
        rospy.loginfo("photo_recorder: ready (topic=%s)", self.sub.resolved_name)

    def cb(self, msg: Image):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("photo_recorder: cv_bridge failed: %s", e)

    def handle(self, _):
        if self.latest is None:
            return TriggerResponse(False, "no image yet")

        now = datetime.now()
        day_dir = os.path.join(self.pkg_root, "object_images",
                               now.strftime("%Y"), now.strftime("%m"), now.strftime("%d"))
        os.makedirs(day_dir, exist_ok=True)
        fname = now.strftime("%H%M%S") + ".jpg"
        path = os.path.join(day_dir, fname)
        try:
            cv2.imwrite(path, self.latest)
        except Exception as e:
            return TriggerResponse(False, "save failed: %s" % e)
        return TriggerResponse(True, fname)

if __name__ == "__main__":
    PhotoRecorder()
    rospy.spin()
