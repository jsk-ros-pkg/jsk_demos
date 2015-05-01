#! /usr/bin/python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('gazebo_drive_simulator')
import rospy, rospkg, cv2, numpy, math, sys
from std_msgs.msg import Float64

im_in = None

def init():
    global im_in
    rospy.init_node("handle_viewer")
    rospy.Subscriber("steering", Float64, callback)
    im_in = cv2.imread(rospkg.RosPack().get_path('gazebo_drive_simulator') + "/config/steering_wheel_image.jpg")
    argc = len(sys.argv)
    if (argc == 2):
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
        if (argc == 2):
            cv2.imshow(sys.argv[1], im_ro)
        else:
            cv2.imshow("Handle", im_ro)
        cv2.waitKey(1)

def rotate_about_center(src, angle, scale=1.0):
    # angle is assumed to be radian
    w = src.shape[1]
    h = src.shape[0]
    # now calculate new image width and height
    nw = (abs(numpy.sin(angle)*h) + abs(numpy.cos(angle)*w))*scale
    nh = (abs(numpy.cos(angle)*h) + abs(numpy.sin(angle)*w))*scale
    # ask OpenCV for the rotation matrix
    rot_mat = cv2.getRotationMatrix2D((nw*0.5, nh*0.5), numpy.rad2deg(angle), scale)
    # calculate the move from the old center to the new center combined
    # with the rotation
    rot_move = numpy.dot(rot_mat, numpy.array([(nw-w)*0.5, (nh-h)*0.5,0]))
    # the move only affects the translation, so update the translation
    # part of the transform
    rot_mat[0,2] += rot_move[0]
    rot_mat[1,2] += rot_move[1]
    return cv2.warpAffine(src, rot_mat, (int(math.ceil(nw)), int(math.ceil(nh))), flags=cv2.INTER_LANCZOS4)
 
if __name__ == "__main__":
    init()
    rospy.spin()
