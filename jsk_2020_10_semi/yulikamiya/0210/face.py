#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

import cv2
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import CompressedImage


class ImageToLabel(ConnectionBasedTransport):
    def __init__(self):
        rospy.set_param('~always_subscribe', True)
        super(ImageToLabel, self).__init__()
        self.subscribe()

    def subscribe(self):
        self._sub = rospy.Subscriber(
            '{}/compressed'.format(rospy.resolve_name('~input')),
            CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)


    def unsubscribe(self):
        self._sub.unregister()

    def image_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = img[:, :, ::-1]
        # do job
        cv2.imshow("image", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_to_label')
    img2label = ImageToLabel()
    rospy.spin()
