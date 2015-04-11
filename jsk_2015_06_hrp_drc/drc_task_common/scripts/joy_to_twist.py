#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)
from sensor_msgs.msg import Joy
from geometry_msgs.msg import *    
import numpy

twist = Twist()

def timer_cb(event):
    pub_twist()
def pub_twist():
    global twist
    twist_pub.publish(twist)
def joy_cb(msg):
    global twist
    # twist.linear.x = msg.axes[1]
    # twist.angular.z = msg.axes[0]
    # twist.angular.y = msg.axes[4]
    # twist.angular.x = msg.axes[3]
    # twist.linear.y = (msg.axes[5] - msg.axes[2])/2.0 + msg.axes[6]
    # twist.linear.z = msg.axes[7] 
    twist.linear.z = msg.buttons[1] - msg.buttons[0] + msg.buttons[3]
    twist.angular.z = -msg.axes[3]
    twist.angular.y = msg.axes[4]
    twist.angular.x = - (msg.axes[5] - msg.axes[2])/2.0 
    twist.linear.y = msg.axes[0]
    twist.linear.x = msg.axes[1] 
    
if __name__ == "__main__":
    rospy.init_node('joy_to_twist', anonymous=True)
    twist_pub = rospy.Publisher('twist', Twist)
    rospy.Subscriber("joy", Joy, joy_cb)
    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()
