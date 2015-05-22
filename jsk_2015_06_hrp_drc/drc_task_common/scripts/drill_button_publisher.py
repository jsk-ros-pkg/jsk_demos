#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *    

if __name__ == "__main__":
    rospy.init_node('drill_button_publisher', anonymous=True)
    button_marker_pub = rospy.Publisher('/button_marker', Marker)
    button_marker = Marker(ns="button", id=0, type=Marker.ARROW, action=Marker.ADD, frame_locked=True)
    button_marker.header = std_msgs.msg.Header(frame_id="drill")
    button_marker.pose = Pose(position=Point(x=-0.022, y=+0.08, z=0.077), orientation=Quaternion(x=0, y=0, z=0.701, w=-0.701))
    button_marker.scale = Vector3(x=0.05, y=0.015, z=0.015)
    button_marker.color = std_msgs.msg.ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.7)

    arrow_marker_array_pub = rospy.Publisher('/axis_marker', MarkerArray)
    arrow_marker_array = MarkerArray()
    arrow_marker = Marker(ns="x", id=1, type=Marker.ARROW, action=Marker.ADD, frame_locked=True)
    arrow_marker.header = std_msgs.msg.Header(frame_id="drill")
    arrow_marker.pose = Pose(orientation=Quaternion(x=0, y=0, z=0, w=1))
    arrow_marker.scale = Vector3(x=0.3, y=0.05, z=0.05)
    arrow_marker.color = std_msgs.msg.ColorRGBA(r=1, g=0, b=0, a=0.7)
    arrow_marker_array.markers.append(arrow_marker)
    arrow_marker = Marker(ns="y", id=1, type=Marker.ARROW, action=Marker.ADD, frame_locked=True)
    arrow_marker.header = std_msgs.msg.Header(frame_id="drill")
    arrow_marker.pose = Pose(orientation=Quaternion(x=0, y=0, z=0.701, w=-0.701))
    arrow_marker.scale = Vector3(x=0.3, y=0.05, z=0.05)
    arrow_marker.color = std_msgs.msg.ColorRGBA(r=0, g=1, b=0, a=0.7)
    arrow_marker_array.markers.append(arrow_marker)
    arrow_marker = Marker(ns="z", id=1, type=Marker.ARROW, action=Marker.ADD, frame_locked=True)
    arrow_marker.header = std_msgs.msg.Header(frame_id="drill")
    arrow_marker.pose = Pose(orientation=Quaternion(x=0, y=0.701, z=0, w=-0.701))
    arrow_marker.scale = Vector3(x=0.3, y=0.05, z=0.05)
    arrow_marker.color = std_msgs.msg.ColorRGBA(r=0, g=0, b=1, a=0.7)
    arrow_marker_array.markers.append(arrow_marker)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        button_marker_pub.publish(button_marker)
        arrow_marker_array_pub.publish(arrow_marker_array)
        r.sleep()
