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
from std_srvs import srv
joy = None
joy_before = None
factor = 0.1
factor_l = 0.15
factor_r = 1.0
mode = 1
r_flag = False
l_flag = False
reverse_flag = False
def timer_cb(event):
    global joy, joy_before, mode
    if joy: # and joy_before and joy_before.header.seq != joy.header.seq:
        twist = joy_to_twist(joy)
        if not check_twist(twist):
            return
        # if mode == 0:
        #     twist_pub.publish(twist)
        if mode == 0:
            pose = twist_to_pose(twist)
            pose_pub.publish(pose)
        if mode == 1:
            pose = twist_to_pose(twist)
            pose_relative_pub.publish(pose)
    joy_before = joy

def check_twist(twist):
    return (not (twist.linear.x == 0.0) & (twist.linear.y == 0.0) & (twist.linear.z == 0.0) & (twist.angular.x == 0.0) & (twist.angular.y == 0.0) & (twist.angular.z == 0.0))
def joy_cb(msg):
    global joy, factor, reverse_flag
    joy = msg
    if msg.buttons[5]:
        factor*=1.2
        print "factor: %f" % factor
    if msg.buttons[4]:
        factor/=1.2
        print "factor: %f" % factor
    if msg.buttons[8]:
        req_marker_default_srv()
    if msg.buttons[7]:
        global mode
        mode += 1
        mode %= 2
        print "mode: %d" % mode
        
    if msg.buttons[6]:
        if reverse_flag == False:
            pose_relative_pub.publish(Pose(orientation=Quaternion(0, 0, 1, 0)))
            reverse_flag = True
    else:
        reverse_flag = False
    
def joy_to_twist(msg):
    twist = Twist()
    axes_list = list (msg.axes)
    # twist.linear.x = axes_list[1]
    # twist.angular.z = axes_list[0]
    # twist.angular.y = axes_list[4]
    # twist.angular.x = axes_list[3]
    # twist.linear.y = (axes_list[5] - axes_list[2])/2.0 + axes_list[6]
    # twist.linear.z = axes_list[7] 
    # twist.linear.z = msg.buttons[1] - msg.buttons[0] + msg.buttons[3] - msg.buttons[2]
    # twist.angular.z = -axes_list[3]
    # twist.angular.y = axes_list[4]
    # twist.angular.x = - (axes_list[5] - axes_list[2])/2.0 
    # twist.linear.y = axes_list[0]
    # twist.linear.x = axes_list[1] 
    # twist.linear.z = msg.buttons[1] - msg.buttons[0] + msg.buttons[3] - msg.buttons[2]
    # twist.angular.y = -axes_list[3]
    # twist.angular.z = -axes_list[4]
    # twist.angular.x = - axes_list[2] 
    # twist.linear.y = axes_list[0]
    # twist.linear.x = axes_list[1]
    global r_flag, l_flag
    if (not r_flag) and axes_list[5]==0.0:
        axes_list[5] = 1.0
    else:
        r_flag = True
    if (not l_flag) and axes_list[2]==0.0:
        axes_list[2] = 1.0
    else:
        l_flag = True
    
    twist.linear.z = axes_list[7] - msg.buttons[0] +msg.buttons[3]
    twist.linear.y = axes_list[0]
    twist.linear.x = axes_list[1] 
    twist.angular.y = axes_list[4]
    twist.angular.x = -axes_list[3]
    twist.angular.z = (axes_list[5] - axes_list[2]) /2.0
    cut_twist(twist)
    factor_twist(twist)
    return twist
def cut_twist(twist):
    if (numpy.abs(twist.linear.x) < 0.25):
        twist.linear.x = 0
    if (numpy.abs(twist.linear.y) < 0.25):
        twist.linear.y = 0
    if (numpy.abs(twist.linear.z) < 0.25):
        twist.linear.z = 0
    if (numpy.abs(twist.angular.x) < 0.25):
        twist.angular.x = 0
    if (numpy.abs(twist.angular.y) < 0.25):
        twist.angular.y = 0
    if (numpy.abs(twist.angular.z) < 0.25):
        twist.angular.z = 0
    
    
def factor_twist(twist):
    global factor, factor_l, factor_r
    twist.linear.x*=(factor * factor_l)
    twist.linear.y*=(factor * factor_l)
    twist.linear.z*=(factor * factor_l)
    twist.angular.x*=(factor * factor_r)
    twist.angular.y*=(factor * factor_r)
    twist.angular.z*=(factor * factor_r)
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
    pose_pub = rospy.Publisher('/transformable_interactive_server/add_pose', Pose)
    pose_relative_pub = rospy.Publisher('/transformable_interactive_server/add_pose_relative', Pose)
    req_marker_default_srv = rospy.ServiceProxy('/set_drill_coords', srv.Empty)
    rospy.Subscriber("joy", Joy, joy_cb)
    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()
