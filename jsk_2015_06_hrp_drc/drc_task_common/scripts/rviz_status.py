#!/usr/bin/env python

import rospy
from std_msgs.msg import Time, ColorRGBA, UInt8
from jsk_rviz_plugins.msg import OverlayText
from drc_com_common.msg import FC2OCSLarge
rospy.init_node("rviz_status")

##########################################
# useful variables
##########################################
OK_COLOR = ColorRGBA(r=91/255.0, g=255/255.0, b=255/255.0, a=1.0)
WARN_COLOR = ColorRGBA(r=255/255.0, g=202/255.0, b=0/255.0, a=1.0)
UNKNOWN_COLOR = ColorRGBA(r=251/255.0, g=221/255.0, b=221/255.0, a=1.0)
TRANSPARENT_COLOR = ColorRGBA(r=0/255.0, g=0/255.0, b=0/255.0, a=0.0)
ROBOT_STATE_BG_COLOR = ColorRGBA(r=140/255.0, g=100/255.0, b=50/255.0, a=0.2)
UPDATED_TIME_BG_COLOR = ColorRGBA(r=115/255.0, g=30/255.0, b=170/255.0, a=0.2)

##########################################
# robot status
# show text about if robot is moving
##########################################
pub_continuous_robot_status = rospy.Publisher("/ocs/rviz/visualization/continuous_robot_status", OverlayText)
def robotStatusBasicInfoCallback(msg):
    text = OverlayText()
    text.bg_color = TRANSPARENT_COLOR
    if msg.data == FC2OCSLarge.ROBOT_IDLE:
        text.text = "NOW: Robot is idle"
        text.fg_color = OK_COLOR
    elif msg.data == FC2OCSLarge.ROBOT_MOVING:
        text.text = "NOW: Robot is moving"
        text.fg_color = WARN_COLOR
    else:
        text.text = "NOW: Robot is in unnknown status"
        text.fg_color = UNKNOWN_COLOR
    text.bg_color = ROBOT_STATE_BG_COLOR
    pub_continuous_robot_status.publish(text)
sub_continuous_robot_status = rospy.Subscriber("/ocs/robot_status", UInt8, robotStatusBasicInfoCallback)

pub_burst_robot_status = rospy.Publisher("/ocs/rviz/visualization/burst_robot_status", OverlayText)
def robotStatusBasicInfoCallback(msg):
    text = OverlayText()
    text.bg_color = TRANSPARENT_COLOR
    if msg.data == FC2OCSLarge.ROBOT_IDLE:
        text.text = "BURST: Robot was idle"
        text.fg_color = OK_COLOR
    elif msg.data == FC2OCSLarge.ROBOT_MOVING:
        text.text = "BURST: Robot was moving"
        text.fg_color = WARN_COLOR
    else:
        text.text = "BURST: Robot was in unnknown status"
        text.fg_color = UNKNOWN_COLOR
    text.bg_color = ROBOT_STATE_BG_COLOR
    pub_burst_robot_status.publish(text)
sub_burst_robot_status = rospy.Subscriber("/ocs/communication/robot_status", UInt8, robotStatusBasicInfoCallback)

##########################################
# communication status
##########################################

pub_continuous_data_updated_time = rospy.Publisher("/ocs/rviz/visualization/continuous_updated_time", OverlayText)
def continuousDataUpdatedTimeCallnack(msg):
    text = OverlayText()
    now = rospy.Time.now()
    diff = (now - msg.data).to_sec()
    text.bg_color = TRANSPARENT_COLOR
    if msg.data.to_sec() == 0:
        text.text = "CONTINUOUS DATA: Not received yet"
        diff = 100
    else:
        text.text = "CONTINUOUS DATA: Updated %0.1f secs before" % (diff)
    if diff <= 1.0:
        text.fg_color = OK_COLOR
    else:
        text.fg_color = WARN_COLOR
    text.bg_color = UPDATED_TIME_BG_COLOR
    pub_continuous_data_updated_time.publish(text)
sub_continuous_data_updated_time = rospy.Subscriber("/ocs_from_fc_basic_low_speed/last_received_time", 
                                                    Time, continuousDataUpdatedTimeCallnack)


pub_lowspeed_data_updated_time = rospy.Publisher("/ocs/rviz/visualization/lowspeed_updated_time", OverlayText)
def lowspeedDataUpdatedTimeCallnack(msg):
    text = OverlayText()
    now = rospy.Time.now()
    diff = (now - msg.data).to_sec()
    text.bg_color = TRANSPARENT_COLOR
    if msg.data.to_sec() == 0:
        text.text = "LOWSPEED DATA: Not received yet"
        diff = 100
    else:
        text.text = "LOWSPEED DATA: Updated %0.1f secs before" % (diff)
    if diff <= 1.0:
        text.fg_color = OK_COLOR
    else:
        text.fg_color = WARN_COLOR
    text.bg_color = UPDATED_TIME_BG_COLOR
    pub_lowspeed_data_updated_time.publish(text)
sub_lowspeed_data_updated_time = rospy.Subscriber("/ocs_from_fc_low_speed/last_received_time", 
                                                    Time, lowspeedDataUpdatedTimeCallnack)


pub_burst_data_updated_time = rospy.Publisher("/ocs/rviz/visualization/burst_updated_time", OverlayText)
def burstDataUpdatedTimeCallnack(msg):
    text = OverlayText()
    now = rospy.Time.now()
    diff = (now - msg.data).to_sec()
    text.bg_color = TRANSPARENT_COLOR
    if msg.data.to_sec() == 0:
        text.text = "BURST DATA: Not received yet"
        diff = 100
    else:
        text.text = "BURST DATA: Updated %0.1f secs before" % (diff)
    if diff <= 30.0:
        text.fg_color = OK_COLOR
    else:
        text.fg_color = WARN_COLOR
    text.bg_color = UPDATED_TIME_BG_COLOR
    pub_burst_data_updated_time.publish(text)
sub_burst_data_updated_time = rospy.Subscriber("/highspeed_receiver/last_received_time", Time,
                                               burstDataUpdatedTimeCallnack)

rospy.spin()
