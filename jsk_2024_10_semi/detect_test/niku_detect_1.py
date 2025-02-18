#!/usr/bin/env python

##切れ目のsegmentation

##参考: https://qiita.com/ysdyt/items/5972c9520acf6a094d90
## https://pystyle.info/opencv-distance-transform/#google_vignette


#import common pkg in Python
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

#import pkg for connecting to ROS
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo


#define call back function
def callback(msg):
    global niku
    niku = msg[0]



rospy.init_node('client')
rospy.Subscriber("/kinect_head/rgb/image_raw/compressed", CompressedImage, callback)
    
niku = cv.imread("./niku.jpg")
niku = niku[1000: 3200, 500 : 2700]


niku_gray = cv.cvtColor(niku, cv.COLOR_BGR2GRAY)



plt.imshow(niku_gray,cmap='gray')
plt.show()

thresh,bin_img = cv.threshold(niku_gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
plt.imshow(bin_img,cmap='gray')
plt.show()
print('大津法の二値化によって自動で決定された閾値:',thresh)

## print(bin_img) #for debug

kernel = np.ones((3,3),np.uint8)
opening = cv.morphologyEx(bin_img,cv.MORPH_OPEN,kernel,iterations = 2)
plt.imshow(opening,cmap='gray')
plt.show()

#モルフォロジー演算のDilationを使う
sure_bg = cv.dilate(opening,kernel,iterations=2)
plt.imshow(sure_bg,cmap='gray')
plt.show()

dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
plt.imshow(dist_transform)
plt.show()

ret, sure_fg = cv.threshold(dist_transform,0.05*dist_transform.max(),255,0)
print('閾値（距離変換で得られた値の最大値×0.5）:',ret)
plt.imshow(sure_fg,cmap='gray')
plt.show()

sure_fg = np.uint8(sure_fg)
unknown = cv.subtract(sure_bg,sure_fg)
plt.imshow(unknown,cmap='gray')

# foregroundの1オブジェクトごとにラベル（番号）を振っていく
ret, markers = cv.connectedComponents(sure_fg)
plt.imshow(markers)
plt.show()
