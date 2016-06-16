#!/usr/bin/env python


import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from threading import Lock
#from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface

g_resolution_lock = Lock()
g_cloud_lock = Lock()
g_resolution_msg = None
g_cloud_msg = None

def resolution_callback(msg):
    global g_resolution_msg, g_resolution_lock
    with g_resolution_lock:
        g_resolution_msg = msg
def cloud_callback(msg):
    global g_cloud_msg, g_cloud_lock
    with g_cloud_lock:
        g_cloud_msg = msg

def publish_text(event):
    global g_resolution_msg, g_resolution_lock, g_cloud_msg, g_cloud_lock
    with g_resolution_lock:
        with g_cloud_lock:
            if not g_cloud_msg or not g_resolution_msg:
                return
            text_interface.publish( """PointCloud Resolution: {0}
Number of Points: {1}""".format(g_resolution_msg.data, g_cloud_msg.width * g_cloud_msg.height))

if __name__ == "__main__":
    rospy.init_node("octree_info")
    # pub = rospy.Publisher("~text", OverlayText)
    text_interface = OverlayTextInterface("~text")
    sub_resolution = rospy.Subscriber("~resolution", Float32, resolution_callback)
    sub_cloud = rospy.Subscriber("~cloud", PointCloud2, cloud_callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
