#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
from visualization_msgs.msg import Marker, MarkerArray


class MarkerIndigo(Marker):
    _md5sum = "18326976df9d29249efc939e00342cde"


class MarkerArrayIndigo(MarkerArray):
    _md5sum = "90da67007c26525f655c1c269094e39f"


class MarkerKinetic(Marker):
    _md5sum = "4048c9de2a16f4ae8e0538085ebf1b97"


class MarkerArrayKinetic(MarkerArray):
    _md5sum = "d155b9ce5188fbaf89745847fd5882d7"


class MarkerBridge(object):
    def __init__(self):
        self.publishers = {}
        self.subscribers = {}
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    def msg_cb(self, msg, pubs):
        cls, cls_indigo, cls_kinetic, pub_indigo, pub_kinetic = pubs

        msg_indigo = cls_indigo()
        msg_kinetic = cls_kinetic()

        for slot in msg_indigo.__slots__:
            setattr(msg_indigo, slot, getattr(msg, slot))
        for slot in msg_kinetic.__slots__:
            setattr(msg_kinetic, slot, getattr(msg, slot))

        pub_indigo.publish(msg_indigo)
        pub_kinetic.publish(msg_kinetic)

    def timer_cb(self, event=None):
        topics = rospy.get_published_topics()
        names = [t[0] for t in topics if t[1] in [Marker._type, MarkerArray._type]]

        # advertise
        for name, _type in topics:
            if name in self.publishers:
                continue
            if name.endswith("/indigo") or name.endswith("/kinetic"):
                continue
            if _type == Marker._type:
                pub_indigo = rospy.Publisher(name + "/indigo", MarkerIndigo, queue_size=1)
                pub_kinetic = rospy.Publisher(name + "/kinetic", MarkerKinetic, queue_size=1)
                self.publishers[name] = (Marker, MarkerIndigo, MarkerKinetic, pub_indigo, pub_kinetic)
            elif _type == MarkerArray._type:
                pub_indigo = rospy.Publisher(name + "/indigo", MarkerArrayIndigo, queue_size=1)
                pub_kinetic = rospy.Publisher(name + "/kinetic", MarkerArrayKinetic, queue_size=1)
                self.publishers[name] = (MarkerArray, MarkerArrayIndigo, MarkerArrayKinetic,  pub_indigo, pub_kinetic)

        for name, pubs in self.publishers.items():
            # unadvertise
            cls, clsi, clsk, pub_indigo, pub_kinetic = pubs
            if name not in names:
                pub_indigo.unregister()
                pub_kinetic.unregister()
                self.publishers.pop(name)

            # subscribe / unsubscribe
            num = pub_indigo.get_num_connections() + pub_kinetic.get_num_connections()
            if num > 0 and name not in self.subscribers:
                sub = rospy.Subscriber(name, cls, self.msg_cb, pubs)
                self.subscribers[name] = sub
            elif num == 0 and name in self.subscribers:
                sub = self.subscribers.pop(name)
                sub.unregister()


if __name__ == '__main__':
    rospy.init_node("marker_bridge")
    MarkerBridge()
    rospy.spin()
