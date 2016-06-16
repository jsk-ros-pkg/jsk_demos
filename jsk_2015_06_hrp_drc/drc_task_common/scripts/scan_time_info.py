#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from threading import Lock
#from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from jsk_recognition_msgs.msg import TimeRange

g_lock = Lock()
g_msg = None

def callback(msg):
    global g_msg, g_lock
    with g_lock:
        g_msg = msg

def publish_text(event):
    global g_lock, g_msg
    with g_lock:
        if not g_msg:
            return
        text_interface.publish( """Scan took {0} secs""".format((g_msg.end - g_msg.start).to_sec()))
        pub.publish(Float32(data=(g_msg.end - g_msg.start).to_sec()))

if __name__ == "__main__":
    rospy.init_node("scan_time_info")
    pub = rospy.Publisher("~time", Float32)
    text_interface = OverlayTextInterface("~text")
    sub = rospy.Subscriber("~range", TimeRange, callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
