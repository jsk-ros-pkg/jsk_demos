#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_srvs import srv
from functools import partial
from sound_play.msg import SoundRequest

def publishModeText(message='test', bg_color=[0.9,0.9,0.9,0.1], fg_color=[0.2,0.2,0.2,0.1]):
    send_text = OverlayText()
    send_text.text = message
    send_text.top = 50
    send_text.left = 25
    send_text.width = 1500
    send_text.height = 200
    send_text.bg_color.r,send_text.bg_color.g,send_text.bg_color.b,send_text.bg_color.a=bg_color
    send_text.fg_color.r,send_text.fg_color.g,send_text.fg_color.b,send_text.fg_color.a=fg_color
    #send_text.line_width = 1
    send_text.text_size = 100
    send_text_pub.publish(send_text)


def publishModeEmergency(message='EMERGENCY'):
    while (not rospy.is_shutdown()) and operation_mode == 'emergency':
        publishModeText(message=message, fg_color=[1,0,0,1])
        r.sleep()
        publishModeText(message=message, fg_color=[1,0,0,0.5])
        r.sleep()

def publishModeTeleoperating(message='Teleoperating'):
    while (not rospy.is_shutdown()) and operation_mode == 'teleoperating':
        publishModeText(message=message, fg_color=[1,0.3,0.1,1])
        r.sleep()
        publishModeText(message=message, fg_color=[1,0.3,0.1,0.5])
        r.sleep()

def publishModeIdling(message='Idling'):
    while (not rospy.is_shutdown()) and operation_mode == 'idling':
        publishModeText(message=message, fg_color=[0,0,1,1])
        r.sleep()

def setOperationMode(req, mode=''):
    global operation_mode
    operation_mode = mode
    talk_command_pub.publish(SoundRequest(arg='Go\ to\ '+mode+'\ mode.'))
    return srv.EmptyResponse()

if __name__ == "__main__":
    global operation_mode
    rospy.init_node("operator_station_state")
    send_text_pub = rospy.Publisher("operator_sation_state_text", OverlayText)
    talk_command_pub = rospy.Publisher('/google_translation_talk_command', SoundRequest)
    rospy.Service('set_operator_mode_emergency', srv.Empty, lambda req: setOperationMode(req, mode='emergency'))
    rospy.Service('set_operator_mode_teleoperating', srv.Empty, lambda req: setOperationMode(req, mode='teleoperating'))
    rospy.Service('set_operator_mode_idling', srv.Empty, lambda req: setOperationMode(req, mode='idling'))
    r = rospy.Rate(2)
    operation_mode = 'tele'
    while not rospy.is_shutdown():
        publishModeEmergency()
        publishModeTeleoperating()
        publishModeIdling()
        r.sleep()

