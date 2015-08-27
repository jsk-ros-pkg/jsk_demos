#!/usr/bin/env bash

source `dirname ${0}`/fc_tmux_init.sh

if [ "$ROBOT" = "JAXON" -o "$ROBOT" = "JAXON_RED" ]; then
    tmux-newwindow multisense "sleep 1; roslaunch jaxon_ros_bridge jaxon_multisense_remote.launch"
fi
tmux-newwindow executive "sleep 1; roslaunch drc_task_common fc_executive.launch"
tmux-newwindow lookat "sleep 1; roslaunch drc_task_common lookat.launch"
tmux-newwindow fisheye "sleep 5; roslaunch drc_task_common fisheye_lookat.launch"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common locomotion.launch"
# tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common locomotion_planner.launch"
tmux-newwindow misc "sleep 5; roslaunch drc_task_common fc_misc.launch"
tmux-newwindow com "sleep 5; roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
