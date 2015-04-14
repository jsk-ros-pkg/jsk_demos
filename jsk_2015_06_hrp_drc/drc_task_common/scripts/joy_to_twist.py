#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
import tf
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)
from sensor_msgs.msg import Joy
from geometry_msgs.msg import *    
import numpy

joy = None
factor = 1.0
mode = 0

def timer_cb(event):
    global joy, mode
    if joy:
        twist = joy_to_twist(joy)
        if not check_twist(twist):
            return
        if mode == 0:
            twist_pub.publish(twist)
        if mode == 1:
            pose = twist_to_pose(twist)
            pose_pub.publish(pose)
        if mode == 2:
            pose = twist_to_pose(twist)
            pose_relative_pub.publish(pose)

def check_twist(twist):
    return (not (twist.linear.x == 0.0) & (twist.linear.y == 0.0) & (twist.linear.z == 0.0) & (twist.angular.x == 0.0) & (twist.angular.y == 0.0) & (twist.angular.z == 0.0))
def joy_cb(msg):
    global joy
    joy = msg
    if msg.buttons[5]:
        global factor
        factor*=1.2
        print "factor: %f" % factor
    if msg.buttons[4]:
        global factor
        factor/=1.2
        print "factor: %f" % factor
    if msg.buttons[12]:
        global mode
        mode += 1
        mode %= 3
        print "mode: %d" % mode
    
def joy_to_twist(msg):
    twist = Twist()
    # twist.linear.x = msg.axes[1]
    # twist.angular.z = msg.axes[0]
    # twist.angular.y = msg.axes[4]
    # twist.angular.x = msg.axes[3]
    # twist.linear.y = (msg.axes[5] - msg.axes[2])/2.0 + msg.axes[6]
    # twist.linear.z = msg.axes[7] 
    # twist.linear.z = msg.buttons[1] - msg.buttons[0] + msg.buttons[3] - msg.buttons[2]
    # twist.angular.z = -msg.axes[3]
    # twist.angular.y = msg.axes[4]
    # twist.angular.x = - (msg.axes[5] - msg.axes[2])/2.0 
    # twist.linear.y = msg.axes[0]
    # twist.linear.x = msg.axes[1] 
    twist.linear.z = msg.buttons[1] - msg.buttons[0] + msg.buttons[3] - msg.buttons[2]
    twist.angular.y = -msg.axes[3]
    twist.angular.z = -msg.axes[4]
    twist.angular.x = - msg.axes[2] 
    twist.linear.y = msg.axes[0]
    twist.linear.x = msg.axes[1] 

    factor_twist(twist)
    return twist
def factor_twist(twist):
    global factor
    twist.linear.x*=factor
    twist.linear.y*=factor
    twist.linear.z*=factor
    twist.angular.x*=factor
    twist.angular.y*=factor
    twist.angular.z*=factor
def twist_to_pose(twist):
    pose = Pose()
    pose.position.x = twist.linear.x
    pose.position.y = twist.linear.y
    pose.position.z = twist.linear.z
    eular = (twist.angular.x, twist.angular.y, twist.angular.z)
    quaternion = tf.transformations.quaternion_from_euler(twist.angular.x, twist.angular.y, twist.angular.z)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose
if __name__ == "__main__":
    rospy.init_node('joy_to_twist', anonymous=True)
    twist_pub = rospy.Publisher('twist', Twist)
    pose_pub = rospy.Publisher('add_pose', Pose)
    pose_relative_pub = rospy.Publisher('add_pose_relative', Pose)
    rospy.Subscriber("joy", Joy, joy_cb)
    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()
