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
def checkFCSilverHammer():
    checkSanityHighspeedStreamer("highspeed_streamer")
    checkSanityLowspeedReceiver("fc_from_ocs_low_speed")
    checkSanityLowspeedStreamer("fc_to_ocs_low_speed")
    checkSanityLowspeedStreamer("fc_to_ocs_basic_low_speed")
    checkSanityLowspeedReceiver("fc_from_ocs_reconfigure")

if __name__ == "__main__":
    rospy.init_node("check_fc_sanity")
    checkFCSilverHammer()
