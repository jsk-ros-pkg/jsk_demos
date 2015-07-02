#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import os
import re

from jsk_tools.sanity_lib import (okMessage, errorMessage, warnMessage, indexMessage,
                                  checkTopicIsPublished,
                                  checkROSMasterCLOSE_WAIT,
                                  checkBlackListDaemon,
                                  checkROSCoreROSMaster,
                                  checkNodeState,
                                  checkSilverHammerSubscribe,
                                  checkGitRepoWithRosPack)
from std_msgs.msg import Time

check_ocs_nodes = ["/b_control_client",
                   "/drill_button_publisher",
                   "/dynamic_tf_publisher_for_urdf_marker",
                   "/footstep_marker",
                   "/footstep_overlay_text",
                   "/footstep_planner",
                   "/footstep_refinement",
                   "/highspeed_receiver",
                   "/highspeed_time_text",
                   "/hrp2jsknts_ik_server",
                   "/image_rect/rqt_image_view2_engine",
                   "/image_rect/scale_background",
                   "/image_rect/scale_foreground",
                   "/image_rect/scale_line",
                   "/image_rect/scale_poly",
                   "/image_rect/scale_screenractangle",
                   "/joint_states_storage",
                   "/joy_to_twist",
                   "/jsk_model_marker_interface",
                   "/low_speed_basic_time_text",
                   "/low_speed_time_text",
                   "/midi_config_player",
                   "/move_marker_with_point",
                   "/ocs/accel_throttle",
                   "/ocs/brake_throttle",
                   "/ocs/dynamic_tf_publisher",
                   "/ocs/env_server",
                   "/ocs/ground_cloud",
                   "/ocs/ground_extraction",
                   "/ocs/ground_polygon_publisher",
                   "/ocs/handle_throttle",
                   "/ocs/joint_states_buffer_client",
                   "/ocs/joy_driver",
                   "/ocs/joy_manager",
                   "/ocs/locomotion_manager",
                   "/ocs/neck_p_throttle",
                   "/ocs/neck_y_throttle",
                   "/ocs/non_ground_points",
                   "/ocs/obstacle_cloud",
                   "/ocs/ocs_robot_state_publisher",
                   "/ocs/plane_concatenator",
                   "/ocs/plane_estimation",
                   "/ocs/vehicle_ui",
                   "/ocs_basic_info_publisher",
                   "/ocs_dynamic_reconfigure",
                   "/ocs_executive",
                   "/ocs_from_fc_basic_low_speed",
                   "/ocs_from_fc_eus",
                   "/ocs_from_fc_low_speed",
                   "/ocs_from_fc_vehicle",
                   "/ocs_to_fc_eus",
                   "/ocs_to_fc_low_speed",
                   "/ocs_to_fc_reconfigure",
                   "/ocs_to_fc_vehicle",
                   "/operator_rviz",
                   "/panorama_view/rqt_panorama_image_view2_engine",
                   "/panorama_view/scale_background",
                   "/panorama_view/scale_foreground",
                   "/panorama_view/scale_line",
                   "/panorama_view/scale_poly",
                   "/panorama_view/scale_screenractangle",
                   "/request_ik_from_marker",
                   "/rosout",
                   "/rqt_image_view_button",
                   "/rviz_menu_server",
                   "/rviz_status",
                   "/spacenav",
                   "/spacenav_client",
                   "/stair_marker",
                   "/stat_visualization/motor_states_temperature_decomposer",
                   "/stat_visualization/robot_name_publisher",
                   "/stat_visualization/temperature_name_publisher",
                   "/state_image_publisher",
                   "/state_viewer",
                   "/task_state_manager",
                   "/transformable_interactive_server",
                   "/urdf_control_marker",
                   "/vehicle_ocs_executive",
                   "/walk_to_object",
                   ]

def main():
    host = re.match("http://([0-9a-zA-Z]*):.*", os.environ["ROS_MASTER_URI"]).groups(0)[0]
    indexMessage("Check Git Repos in FC")
    checkGitRepoWithRosPack("drc_task_common")
    checkGitRepoWithRosPack("jsk_tools")

    indexMessage("Check BlacklistDaemons in OCS")
    checkBlackListDaemon(["chrome", "dropbox", "skype"], kill=True)

    indexMessage("Check Master in OCS Network")
    # checkROSCoreROSMaster()
    checkROSMasterCLOSE_WAIT(host)
    rospy.init_node("check_sanity_ocs")

    indexMessage("Check Nodes in OCS")
    # check Node State
    for node in check_ocs_nodes:
        checkNodeState(node, needed=True)

    indexMessage("Check Input Device")
    # check Input Device
    checkNodeState('/midi_config_player', needed=True, sub_fail="Is MIDI connected ? or Is that powered on?")
    checkTopicIsPublished("/ocs/joy", None,
                          "XBox Controller seems to work",
                          "XBox Controller doesn't publish. Did you connect?")

    checkTopicIsPublished(
        "/ocs_dynamic_reconfigure/parameter_descriptions",
        Time,
        "[ocs] Silverhammer lowspeed protocol is working",
        "[ocs] Silverhammer lowspeed protocol is not working",
        5,
        [
            ["/ocs_from_fc_basic_low_speed/last_received_time", Time],
            ["/ocs_from_fc_eus/last_received_time", Time],
            ["/ocs_from_fc_low_speed/last_received_time", Time],
            ["/ocs_from_fc_vehicle/last_received_time", Time],
            ["/ocs_to_fc_eus/last_send_time", Time],
            ["/ocs_to_fc_low_speed/last_send_time", Time],
            ["/ocs_to_fc_reconfigure/last_send_time", Time],
            ["/ocs_to_fc_vehicle/last_send_time", Time],
        ])

    indexMessage("Check SilverHammer Subscribe Hz in OCS")
    checkSilverHammerSubscribe("/ocs_to_fc_eus/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/ocs_to_fc_low_speed/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/ocs_to_fc_vehicle/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/ocs_to_fc_reconfigure/last_send_time", 1.0, 0.4, timeout=7)
    checkSilverHammerSubscribe("/ocs_from_fc_basic_low_speed/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/ocs_from_fc_eus/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/ocs_from_fc_low_speed/last_received_time", 10.0, 8.0, timeout=7)
    checkSilverHammerSubscribe("/ocs_from_fc_vehicle/last_received_time", 10.0, 8.0, timeout=7)

if __name__ == "__main__":
    main()
