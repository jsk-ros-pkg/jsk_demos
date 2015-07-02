#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
import numpy as np
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)
from drc_task_common.msg import Float32ArrayStamped
from drc_task_common.srv import *

count = 0
N = 64
def fft_cb(fft_msg):
    fft_array = np.array(fft_msg.data)
    normal1 = np.linalg.norm(fft_array[(N/4*3):(N)])
    normal2 = np.linalg.norm(fft_array[0:N/4*3])
    global count
    ##rospy.loginfo("normal1, %f normal2, %f" % (normal1, normal2))
    if (normal1 > 8):
        ##rospy.loginfo("button pushed! normal %f" % normal)
        if (count < 1000):
            count = count + 1
    else:
        
        ##rospy.loginfo("fail push nomal %f" % normal)
        count = 0
def get_button_state(req):
    success = False
    if (count > 10):
        success = True
    return GetBoolStateResponse(success)
if __name__ == "__main__":
    rospy.init_node('button_checker', anonymous=True)
    rospy.Subscriber("input", Float32ArrayStamped, fft_cb)
    rospy.Service("get_button_state", GetBoolState, get_button_state)
    rospy.spin()
