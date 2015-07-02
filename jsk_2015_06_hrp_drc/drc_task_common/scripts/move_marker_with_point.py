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
import numpy
import tf
from jsk_interactive_marker.srv import *
def point_cb(point_msg):
    try:
        pose = get_pose("").pose_stamped
    except rospy.ServiceException, e:
        print "Service fail: %s" % e
        return
    try:
        transed_point = listener.transformPoint(pose.header.frame_id, point_msg)
        pose.pose.position = transed_point.point
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print "tf error: %s" % e
        return
    try:
        set_pose(target_name="", pose_stamped=pose)
    except rospy.ServiceException, e:
        print "Service fail: %s" % e
        return

if __name__ == "__main__":
    rospy.init_node('revice_button_pose', anonymous=True)
    #posepub = rospy.Publisher('', PoseStamped)
    get_pose = rospy.ServiceProxy('/transformable_interactive_server/get_control_pose', GetTransformableMarkerPose)
    set_pose = rospy.ServiceProxy('/transformable_interactive_server/set_control_pose', SetTransformableMarkerPose)
    listener = tf.TransformListener()
    rospy.Subscriber("/clicked_point", PointStamped, point_cb)
    rospy.spin()
