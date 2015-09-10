#!/usr/bin/env bash

source `dirname ${0}`/ocs_tmux_init.sh

tmux-newwindow executive "roslaunch drc_task_common ocs_executive.launch ocs:=false"
tmux-newwindow ui "roslaunch drc_task_common ui.launch INPUT_IMAGE:=/multisense/left/image_raw"
tmux-newwindow rviz "roslaunch drc_task_common ocs_rviz.launch ik_server_launch:=${ROBOT,,}-ik-server.launch ocs:=false"
# tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common ocs_locomotion.launch"
# tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common ocs_locomotion_planner.launch"
tmux send-keys -t ocs:tmp "exit" C-m
tmux a -t ocs
