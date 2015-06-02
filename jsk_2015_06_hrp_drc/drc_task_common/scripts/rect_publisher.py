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
from std_msgs.msg import *    

if __name__ == "__main__":
    rospy.init_node('rect_publisher', anonymous=True)
    rect_pub = rospy.Publisher('rect_stamped', PolygonStamped)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        rect_pub.publish(PolygonStamped(header=Header(stamp=rospy.Time.now()), polygon=Polygon([Point32(256, 136, 0), Point32(768, 408, 0)])))
        r.sleep()

