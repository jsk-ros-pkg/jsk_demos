#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
from cv_bridge import CvBridge
import rospy
import message_filters
from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


class BoundingBoxCentroid3D(ConnectionBasedTransport):
    def __init__(self):
        super(BoundingBoxCentroid3D, self).__init__()
        self.approximate_sync = rospy.get_param("~approximate_sync", True)
        self.queue_size = rospy.get_param("~queue_size", 10)
        self.slop = rospy.get_param("~slop", 0.1)
        self.buff_size = rospy.get_param("~buff_size", 2**28)
        self.cv_bridge = CvBridge()
        self.camera_info_msg = None
        self.pub_pose = self.advertise("~output", PoseArray, queue_size=1)

    def subscribe(self):
        self.subscribers = [
            message_filters.Subscriber(
                "~input", RectArray,
                queue_size=1, buff_size=self.buff_size),
            message_filters.Subscriber(
                "~input/depth", Image,
                queue_size=1, buff_size=self.buff_size),
        ]
        if self.approximate_sync:
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subscribers, queue_size=self.queue_size, slop=self.slop)
        else:
            sync = message_filters.TimeSynchronizer(
                self.subscribers, queue_size=self.queue_size)
        sync.registerCallback(self.callback)

        self.sub_cam_info = rospy.Subscriber(
                "~input/camera_info", CameraInfo, self.cam_cb,
                queue_size=1, buff_size=self.buff_size)

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()
        self.sub_cam_info.unregister()

    def cam_cb(self, msg):
        self.camera_info_msg = msg
        self.sub_cam_info.unregister()

    def callback(self, rects, depth_msg):
        if self.camera_info_msg is None:
            rospy.logwarn("Camera info is not yet received! Please publish camera info first!")
            return

        rospy.logdebug("callback: %s sec delayed" % (rospy.Time.now() - rects.header.stamp).to_sec())

        try:
            depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except:
            rospy.logerr("Failed to convert depth image")
            return

        if depth_msg.encoding == "16UC1":
            depth = np.asarray(depth, dtype=np.float32)
            depth /= 1000.0
        elif depth_msg.encoding != "32FC1":
            rospy.logerr("Unsupported depth image encoding: %s" % depth_msg.encoding)
            return

        fx = self.camera_info_msg.K[0]
        fy = self.camera_info_msg.K[4]
        cx = self.camera_info_msg.K[2]
        cy = self.camera_info_msg.K[5]

        msg = PoseArray(header=rects.header)
        for r in rects.rects:
            x = r.x + r.width / 2.0
            y = r.y + r.height / 2.0
            z = float(depth[int(np.round(y))][int(np.round(x))])
            if np.isnan(z):
                z = 0.0
            x = (x - cx) * z / fx
            y = (y - cy) * z / fy

            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1
            msg.poses.append(p)

        self.pub_pose.publish(msg)


if __name__ == '__main__':
    rospy.init_node("bounding_box_centroid_3d")
    bbc3d = BoundingBoxCentroid3D()
    rospy.spin()
