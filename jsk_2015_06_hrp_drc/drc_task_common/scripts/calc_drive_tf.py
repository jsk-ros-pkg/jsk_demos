#!/usr/bin/env python
import rospy

PKG='drc_task_common'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import numpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from dynamic_tf_publisher.srv import SetDynamicTF

def pose_cb(pose_msg):
    trans = TransformStamped()
    trans.header = pose_msg.header
    trans.child_frame_id = "handle"
    trans.transform.rotation = pose_msg.pose.orientation
    trans.transform.translation.x = pose_msg.pose.position.x
    trans.transform.translation.y = pose_msg.pose.position.y
    trans.transform.translation.z = pose_msg.pose.position.z
    set_tf(10, trans)
    
if __name__ == "__main__":
    rospy.init_node('calc_drive_tf', anonymous=True)
    # posepub = rospy.Publisher('plane_centroid_pose', PoseStamped)
    set_tf = rospy.ServiceProxy('set_dynamic_tf', SetDynamicTF)
    rospy.Subscriber("pose", PoseStamped,  pose_cb)
    rospy.spin()
