#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
import scipy.fftpack
import numpy as np
import time
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import numpy
import csv
from drc_task_common.msg import Float32ArrayStamped

flug = None
spamWriter = None
def save_cb(float_msg):
    global flug
    if flug:
        print("save cb driven %s" % flug)
        spamWriter.writerow([flug]+list(float_msg.data))
        flug = None
if __name__ == "__main__":
    rospy.init_node('fft_data', anonymous=True)
    spamWriter = csv.writer(open('fft_data.csv', 'wb'), delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    rospy.Subscriber("input", Float32ArrayStamped,  save_cb)
    while not rospy.is_shutdown():
        global flug
        input_string = raw_input("class num: ")
        if(input_string == "exit"):
            exit()
        if(not input_string.isdigit()):
            print("input num only")
            next
        flug = input_string
        time.sleep(1)
