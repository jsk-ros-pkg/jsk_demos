#!/usr/bin/env python

import numpy
import rospy
import imp
imp.find_module('sensor_msgs')
from sensor_msgs.msg import Joy
from geometry_msgs.msg import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from jsk_rviz_plugins.msg import OverlayText

prev_b0 = 0
prev_b1 = 0
direction_mode_list = ['Both', 'Translate', 'Rotate']
direction_mode = direction_mode_list[0]
step_mode = False
relative_mode = False

def callback(msg):
    global step_mode, direction_mode_list, direction_mode, relative_mode
    global prev_b0, prev_b1

    a = msg.axes
    b = msg.buttons

    if prev_b0 == 0 and b[0] == 1:
        relative_mode = not relative_mode
        # direction_mode = direction_mode_list[((direction_mode_list.index(direction_mode)+1) % len(direction_mode_list))]
    if prev_b1 == 0 and b[1] == 1:
        step_mode = not step_mode
    prev_b0 = b[0]
    prev_b1 = b[1]

    x_v,y_v,z_v,rx_v,ry_v,rz_v = [i*j for i,j in zip(a,a_max)]

    target_pose = Pose()
    if direction_mode == 'Both' or direction_mode == 'Translate':
        if step_mode:
            x_v = getStepValue(x_v)
            y_v = getStepValue(y_v)
            z_v = getStepValue(z_v, thre=0.002)
        target_pose.position.x = x_v
        target_pose.position.y = y_v
        target_pose.position.z = z_v

    target_pose.orientation.w = 1
    if direction_mode == 'Both' or direction_mode == 'Rotate':
        if step_mode:
            rx_v = getStepValue(rx_v, ret=numpy.pi/6)
            ry_v = getStepValue(ry_v, ret=numpy.pi/6)
            rz_v = getStepValue(rz_v, ret=numpy.pi/6)
        q = quaternion_from_euler(rx_v, ry_v, rz_v, 'rxyz')
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

    if not [x_v,y_v,z_v,rx_v,ry_v,rz_v]==[0]*6:
        if relative_mode:
            set_pose_relative_pub.publish(target_pose)
        else:
            set_pose_pub.publish(target_pose)
        if step_mode:
            rospy.sleep(0.5)
    if step_mode:
        step_mode_text = 'Step'
    else:
        step_mode_text = 'Continuos'
    if relative_mode:
        relative_mode_text = 'Relative'
    else:
        relative_mode_text = 'Absolute'
    publishModeText('SpaceNav: '+relative_mode_text+' / '+step_mode_text)
    # publishModeText('SpaceNav: '+direction_mode+' / '+step_mode_text)

def getStepValue(val, thre=0.004, ret=0.1):
    if numpy.abs(val) > thre:
        return numpy.sign(val)*ret
    else:
        return 0

def publishModeText(message):
    send_text = OverlayText()
    send_text.text = message
    send_text.top = 130
    send_text.left = 10
    send_text.width = 750
    send_text.height = 50

    send_text.bg_color.r = 0.9
    send_text.bg_color.b = 0.9
    send_text.bg_color.g = 0.9
    send_text.bg_color.a = 0.1
    send_text.fg_color.r = 0.8
    send_text.fg_color.g = 0.8
    send_text.fg_color.b = 0.8
    send_text.fg_color.a = 1
    send_text.line_width = 1
    send_text.text_size = 30
    send_text_pub.publish(send_text)


if __name__ == "__main__":
    rospy.init_node("spacenav_client")
    ns = rospy.get_param('~transformable_interactive_server_nodename', '')

    x_max = rospy.get_param("~x_max", 0.01)
    y_max = rospy.get_param("~y_max", 0.01)
    z_max = rospy.get_param("~z_max", 0.01)
    rx_max = rospy.get_param("~rx_max", 0.01)
    ry_max = rospy.get_param("~ry_max", 0.01)
    rz_max = rospy.get_param("~rz_max", 0.01)
    a_max = [x_max, y_max, z_max, rx_max, ry_max, rz_max]

    set_pose_pub = rospy.Publisher(ns+"/add_pose", Pose)
    set_pose_relative_pub = rospy.Publisher(ns+"/add_pose_relative", Pose)
    send_text_pub = rospy.Publisher("spacenav_client_text", OverlayText)

    s = rospy.Subscriber("input_joy", Joy, callback, queue_size=1)
    rospy.spin()

