#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from geometry_msgs.msg import *
from posedetection_msgs.msg import *

def object_cb(msg):
    if (len(msg.objects) == 0):
        return
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.pose = msg.objects[0].pose
    posepub.publish(pose_msg)
if __name__ == "__main__":
    rospy.init_node('interpret_object', anonymous=True)
    posepub = rospy.Publisher('object_pose', PoseStamped)
    rospy.Subscriber("ObjectDetection", ObjectDetection,  object_cb)
    rospy.spin()
