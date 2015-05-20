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

def fft_cb(fft_msg):
    fft_array = np.array(fft_msg.data)
    normal1 = np.linalg.norm(fft_array[96:128])
    normal2 = np.linalg.norm(fft_array[0:95])
    global count
    if (normal1 > normal2 * 0.5):
        ##rospy.loginfo("button pushed! normal %f" % normal)
        if (count < 1000):
            count = count + 1
    else:
        ##rospy.loginfo("fail push nomal %f" % normal)
        count = 0
def get_button_state(req):
    success = False
    if (count > 200):
        success = True
    return GetBoolStateResponse(success)
if __name__ == "__main__":
    rospy.init_node('button_checker', anonymous=True)
    rospy.Subscriber("input", Float32ArrayStamped, fft_cb)
    rospy.Service("get_button_state", GetBoolState, get_button_state)
    rospy.spin()
