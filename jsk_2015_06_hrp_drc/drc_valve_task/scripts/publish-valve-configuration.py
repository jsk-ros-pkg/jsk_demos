#!/usr/bin/env python

import sys
import roslib
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
pkg = 'jsk_teleop_joy'

import imp
try:
    imp.find_module(pkg)
except:
    roslib.load_manifest(pkg)

import jsk_teleop_joy
from jsk_teleop_joy.nanokontrol_status import NanoKONTROL2Status

thick_abs = 0.0
radius_abs = 0.0
thick_rel = 0.0
radius_rel = 0.0

def change_valve_configuration_cb(msg):
    status = NanoKONTROL2Status(msg)
    global thick_abs, thick_rel, radius_abs, radius_rel
    # thick_abs = float(100 * status.rotate1)
    # if float(status.slide1) > 0.8:
    #     thick_rel += 10.0
    # elif float(status.slide1) < 0.2:
    #     thick_rel += -10.0
    # radius_abs = float(500 * status.rotate2)
    # if float(status.slide2) > 0.8:
    #     radius_rel += 10.0
    # elif float(status.slide2) < 0.2:
    #     radius_rel += -10.0
    thick_abs = 200 * status.rotate1 * status.slide1
    radius_abs = 1000 * status.rotate2 * status.slide2
    thick_pub.publish(Float32(thick_abs + thick_rel))
    radius_pub.publish(Float32(radius_abs + radius_rel))
    if status.prev and status.next and status.play:
        go_pos_pub.publish(Bool(True))
    if status.rec:
        update_eus_pub.publish(Bool(True))

def set_valve_model_radius_cb(msg):
    global thick_abs, thick_rel, radius_abs, radius_rel
    radius_abs = msg.data
    radius_rel = 0
    thick_pub.publish(Float32(thick_abs + thick_rel))
    radius_pub.publish(Float32(radius_abs + radius_rel))

def publish_valve_configuration():
    from sensor_msgs.msg import Joy

    rospy.sleep(1)
    rospy.init_node('nanokontrol_controller')
    s = rospy.Subscriber('/nanokontrol/joy', Joy, change_valve_configuration_cb)
    s = rospy.Subscriber('/set_valve_model_radius', Float32, set_valve_model_radius_cb)
    global thick_pub, radius_pub, go_pos_pub, update_eus_pub
    thick_pub = rospy.Publisher('/valve_model_thick', Float32)
    radius_pub = rospy.Publisher('/valve_model_radius', Float32)
    go_pos_pub = rospy.Publisher('/go_pos_command', Bool)
    update_eus_pub = rospy.Publisher('/update_eus_model', Bool)

    rospy.spin()

if __name__ == '__main__':
    publish_valve_configuration()




