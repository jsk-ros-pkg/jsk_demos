#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from drc_task_common.cfg import BControlDummyServerConfig
from std_srvs.srv import *

def reverse_f(value):
    if value < 0.5:
        return 1.0
    else:
        return 0.0
class BControlDummyServerNode():
    def __init__(self):
        rospy.init_node("b_control_dummy_server")
        r = rospy.Rate(10)
        self.joy_pub = rospy.Publisher("~output", Joy)
        self.joy = Joy(axes=[0.0]*56, buttons=[0]*10)
        rospy.Service('/dummy_u1', Empty, self.u1_cb)
        rospy.Service('/dummy_u2', Empty, self.u2_cb)
        rospy.Service('/dummy_u3', Empty, self.u3_cb)
        rospy.Service('/dummy_u4', Empty, self.u4_cb)
        rospy.Service('/dummy_u5', Empty, self.u5_cb)
        rospy.Service('/dummy_u6', Empty, self.u6_cb)
        rospy.Service('/dummy_u7', Empty, self.u7_cb)
        rospy.Service('/dummy_u8', Empty, self.u8_cb)
        rospy.Service('/dummy_b1', Empty, self.b1_cb)
        rospy.Service('/dummy_b2', Empty, self.b2_cb)
        rospy.Service('/dummy_b3', Empty, self.b3_cb)
        rospy.Service('/dummy_b4', Empty, self.b4_cb)
        rospy.Service('/dummy_b5', Empty, self.b5_cb)
        rospy.Service('/dummy_b6', Empty, self.b6_cb)
        rospy.Service('/dummy_b7', Empty, self.b7_cb)
        rospy.Service('/dummy_b8', Empty, self.b8_cb)
        self.srv = Server(BControlDummyServerConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        self.joy.axes[24] = config.slide1
        self.joy.axes[25] = config.slide2
        self.joy.axes[26] = config.slide3
        self.joy.axes[27] = config.slide4
        self.joy.axes[28] = config.slide5
        self.joy.axes[29] = config.slide6
        self.joy.axes[30] = config.slide7 
        self.joy.axes[31] = config.slide8
        self.joy_pub.publish(self.joy)
        return config

    def pub_joy(self):
        print self.joy
        self.joy_pub.publish(self.joy)
    def u1_cb(self, req):
        self.joy.axes[8] = reverse_f(self.joy.axes[8])
        self.pub_joy()
        return EmptyResponse()
    def u2_cb(self, req):
        self.joy.axes[9] = reverse_f(self.joy.axes[9])
        self.pub_joy()
        return EmptyResponse()
    def u3_cb(self, req):
        self.joy.axes[10] = reverse_f(self.joy.axes[10])
        self.pub_joy()
        return EmptyResponse()
    def u4_cb(self, req):
        self.joy.axes[11] = reverse_f(self.joy.axes[11])
        self.pub_joy()
        return EmptyResponse()
    def u5_cb(self, req):
        self.joy.axes[12] = reverse_f(self.joy.axes[12])
        self.pub_joy()
        return EmptyResponse()
    def u6_cb(self, req):
        self.joy.axes[13] = reverse_f(self.joy.axes[13])
        self.pub_joy()
        return EmptyResponse()
    def u7_cb(self, req):
        self.joy.axes[14] = reverse_f(self.joy.axes[14])
        self.pub_joy()
        return EmptyResponse()
    def u8_cb(self, req):
        self.joy.axes[15] = reverse_f(self.joy.axes[15])
        self.pub_joy()
        return EmptyResponse()
    def b1_cb(self, req):
        self.joy.axes[16] = reverse_f(self.joy.axes[16])
        self.pub_joy()
        return EmptyResponse()
    def b2_cb(self, req):
        self.joy.axes[17] = reverse_f(self.joy.axes[17])
        self.pub_joy()
        return EmptyResponse()
    def b3_cb(self, req):
        self.joy.axes[18] = reverse_f(self.joy.axes[18])
        self.pub_joy()
        return EmptyResponse()
    def b4_cb(self, req):
        self.joy.axes[19] = reverse_f(self.joy.axes[19])
        self.pub_joy()
        return EmptyResponse()
    def b5_cb(self, req):
        self.joy.axes[20] = reverse_f(self.joy.axes[20])
        self.pub_joy()
        return EmptyResponse()
    def b6_cb(self, req):
        self.joy.axes[21] = reverse_f(self.joy.axes[21])
        self.pub_joy()
        return EmptyResponse()
    def b7_cb(self, req):
        self.joy.axes[22] = reverse_f(self.joy.axes[22])
        self.pub_joy()
        return EmptyResponse()
    def b8_cb(self, req):
        self.joy.axes[23] = reverse_f(self.joy.axes[23])
        self.pub_joy()
        return EmptyResponse()
    
if __name__ == '__main__':
    try:
        b_c_n = BControlDummyServerNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
