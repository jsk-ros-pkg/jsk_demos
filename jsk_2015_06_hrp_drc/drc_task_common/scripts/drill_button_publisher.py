#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from visualization_msgs.msg import Marker
from geometry_msgs.msg import *    

if __name__ == "__main__":
    rospy.init_node('drill_button_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/button_marker', Marker)
    marker = Marker(ns="button", id=0, type=Marker.ARROW, action=Marker.ADD, frame_locked=True)
    marker.header = std_msgs.msg.Header(frame_id="drill")
    marker.pose = Pose(position=Point(x=-0.022, y=+0.08, z=0.097), orientation=Quaternion(x=0, y=0, z=0.701, w=-0.701))
    marker.scale = Vector3(x=0.05, y=0.015, z=0.015)
    marker.color = std_msgs.msg.ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.7)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        r.sleep()
