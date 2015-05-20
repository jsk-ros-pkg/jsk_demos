#!/usr/bin/env python

import rospy
import sys
import os
import math
from threading import Lock
from colorama import Fore, Style
from sensor_msgs.msg import Image, JointState, Imu
from jsk_topic_tools.master_util import isMasterAlive
from trans_ros_bridge.msg import ExtraMotorState
from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage,
                                  checkTopicIsPublished,
                                  isMasterHostAlive,
                                  checkIMU)
from jsk_network_tools.silverhammer_util import (checkSanityLowspeedReceiver,
                                                 checkSanityLowspeedStreamer,
                                                 checkSanityHighspeedReceiver,
                                                 checkSanityHighspeedStreamer)
def checkOCSSilverHammer():
    checkSanityHighspeedReceiver("highspeed_receiver")
    checkSanityLowspeedReceiver("ocs_from_fc_low_speed")
    checkSanityLowspeedReceiver("ocs_from_fc_basic_low_speed")
    checkSanityLowspeedStreamer("ocs_to_fc_low_speed")
    checkSanityLowspeedStreamer("ocs_to_fc_reconfigure")

if __name__ == "__main__":
    rospy.init_node("check_ocs_sanity")
    checkOCSSilverHammer()
