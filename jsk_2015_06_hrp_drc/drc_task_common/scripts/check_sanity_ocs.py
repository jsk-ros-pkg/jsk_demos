#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import os
import re

from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage, indexMessage,
                                  checkTopicIsPublished,
                                  checkROSMasterCLOSE_WAIT,
                                  checkNodeState)

from std_msgs.msg import Time

check_ocs_nodes = ["chesk_sanity_fc",
                   "dammy_tf_broadcaster",
                   "rviz_status",
                   "ocs_dynamic_reconfigure",
                   "button_checker",
                   "stair_marker",
                   'calc_box_plane',
                   "spacenav_client",
                   "robot_idle_watch",
                   "time_update_text",
                   'interpret_object',
                   "filter_bbox_position",
                   'b_control_client',
                   'calc_drive_tf',
                   "offset_bbox_from_plane",
                   "static_transform_bounding_box_array",
                   'wrench_fft',
                   'joy_to_twist',
                   "fc_dynamic_reconfigure",
                   'fft_data',
                   'drill_button_publisher',
                   "ar_pose_to_pose",
                   'fft_data',
                   'polygon_to_centroid',
                   "CarPathVisualizer",
                   "magnetometer_to_direction",
                   "SteeringAngleMarker",
                   "google_translation_talk",
                   ]

def main():
    rospy.init_node("check_sanity_ocs")

    host = re.match("http://([0-9a-zA-Z]*):.*", os.environ["ROS_MASTER_URI"]).groups(0)[0]
    checkROSMasterCLOSE_WAIT(host)

    indexMessage("Check Nodes in OCS")
    checkNodeState("/fc_to_ocs_basic_low_speed", True)
    checkNodeState("/fc_to_ocs_eus", True)
    checkNodeState("/fc_to_ocs_low_speed", True)
    checkNodeState("/fc_to_ocs_vehicle", True)
    checkNodeState("/fc_from_ocs_eus", True)
    checkNodeState("/fc_from_ocs_low_speed", True)
    checkNodeState("/fc_from_ocs_vehicle", True)
    checkNodeState("/fc_from_ocs_reconfigure", True)
    checkNodeState("/highspeed_streamer", True)

    indexMessage("Check Nodes in OCS")

    #Check Input Device
    checkTopicIsPublished("/b_control/joy", None,
                          "MIDI Controller seems to work",
                          "MIDI Controller doesn't publish. Does you connect?")
    checkTopicIsPublished("/ocs/joy", None,
                          "XBox Controller seems to work",
                          "XBox Controller doesn't publish. Did you connect?")

    checkTopicIsPublished(
        "/fc_from_ocs_eus/last_publish_output_time",
        Time,
        "[ocs] Silverhammer lowspeed protocol is working",
        "[ocs] Silverhammer lowspeed protocol is not working",
        5,
        [["/fc_from_ocs_eus/last_received_time", Time],
         ["/fc_from_ocs_eus/output", Time],
         ["/fc_from_ocs_low_speed/last_publish_output_time", Time],
         ["/fc_from_ocs_low_speed/last_received_time", Time],
         ["/fc_from_ocs_low_speed/output", Time],
         ["/fc_from_ocs_reconfigure/last_publish_output_time", Time],
         ["/fc_from_ocs_reconfigure/last_received_time", Time],
         ["/fc_from_ocs_reconfigure/output", Time],
         ["/fc_from_ocs_vehicle/last_publish_output_time", Time],
         ["/fc_from_ocs_vehicle/last_received_time", Time],
         ["/fc_from_ocs_vehicle/output", Time],
         ["/fc_to_ocs_basic_low_speed/input", Time],
         ["/fc_to_ocs_basic_low_speed/last_input_received_time", Time],
         ["/fc_to_ocs_basic_low_speed/last_send_time", Time],
         ["/fc_to_ocs_eus/input", Time],
         ["/fc_to_ocs_eus/last_input_received_time", Time],
         ["/fc_to_ocs_eus/last_send_time", Time],
         ["/fc_to_ocs_low_speed/input", Time],
         ["/fc_to_ocs_low_speed/last_input_received_time", Time],
         ["/fc_to_ocs_low_speed/last_send_time", Time],
         ["/fc_to_ocs_vehicle/input", Time],
         ["/fc_to_ocs_vehicle/last_input_received_time", Time],
         ["/fc_to_ocs_vehicle/last_send_time", Time]])

    # check Node State
    for node in check_ocs_nodes:
        checkNodeState(node, needed=True)

if __name__ == "__main__":
    main()
