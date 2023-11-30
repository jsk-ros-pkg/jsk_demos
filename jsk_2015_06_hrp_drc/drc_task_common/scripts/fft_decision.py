#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
import scipy.fftpack
import numpy as np
import csv
from sklearn import svm
from drc_task_common.msg import Float32ArrayStamped


## import message_filters
def load_data(filename):
    spamReader = csv.reader(open(filename, 'rb'), delimiter=' ', quotechar='|')
    X = []
    Y = []
    for row in spamReader:
        X.append(int(row[0]))
        Y.append([float(x) for x in row[1:]])
    return (X, Y)
def disc_cb(msg):
    print(clf.predict(msg.data))
if __name__ == "__main__":
    rospy.init_node('fft_data', anonymous=True)
    x, y = load_data("fft_data.csv")
    print("tests")
    print(x)
    print(y)
    clf = svm.SVC(kernel="rbf")
    clf.fit(y, x)
    rospy.Subscriber("input", Float32ArrayStamped, disc_cb)
    rospy.spin()
