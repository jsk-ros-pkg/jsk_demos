#! /usr/bin/python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('gazebo_drive_simulator')
import rospy, rospkg, cv2, math, numpy, sys
from std_msgs.msg import Float64

im_in = None

def init():
    global im_in
    rospy.init_node("handle_viewer")
    rospy.Subscriber("steering", Float64, callback)
    im_in = cv2.imread(rospkg.RosPack().get_path('gazebo_drive_simulator') + "/config/steering_wheel_image.png")
    argc = len(sys.argv)

    if (argc == 5):
        cv2.imshow(sys.argv[1], im_in)
    else:
        cv2.imshow("Handle", im_in)
    cv2.waitKey(0)
    
def callback(msg):
    global im_in
    if im_in != None:
        angle = msg.data
        im_ro = rotate_about_center(im_in, angle)
        argc = len(sys.argv)
        if (argc == 5):
            cv2.imshow(sys.argv[1], im_ro)
        else:
            cv2.imshow("Handle", im_ro)
        cv2.waitKey(1)

def rotate_about_center(src, angle, scale=1.0):
    # angle is assumed to be radian

    w = src.shape[1]
    h = src.shape[0]
    # ask OpenCV for the rotation matrix
    rot_mat = cv2.getRotationMatrix2D((h*0.5, w*0.5), numpy.rad2deg(angle), scale)    
    
    return cv2.warpAffine(src, rot_mat, (h, w), flags=cv2.INTER_LINEAR)
 
if __name__ == "__main__":
    init()
    rospy.spin()
