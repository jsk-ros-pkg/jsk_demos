#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import RectArray
from opencv_apps.msg import FaceArrayStamped, Face


class RectArrayToFaceArray(ConnectionBasedTransport):
    def __init__(self):
        super(RectArrayToFaceArray, self).__init__()

        self.pub = self.advertise("~output", FaceArrayStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber("~input", RectArray, self.callback, queue_size=1)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):
        pubmsg = FaceArrayStamped(header=msg.header)
        for r in msg.rects:
            face = Face()
            face.face.x = r.x
            face.face.y = r.y
            face.face.width = r.width
            face.face.height = r.height
            pubmsg.faces.append(face)
        self.pub.publish(pubmsg)

if __name__ == '__main__':
    rospy.init_node("rect_array_to_face_array")
    r = RectArrayToFaceArray()
    rospy.spin()
