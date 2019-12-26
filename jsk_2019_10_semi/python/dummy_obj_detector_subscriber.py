#!/usr/bin/env python

import rospy
from posedetection_msgs.msg import ObjectDetection

def cb(msg):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node("dummy_obj_detector_subscriber", anonymous=True)

    rospy.Subscriber("~input/dummy", ObjectDetection, cb)

    rospy.spin()
