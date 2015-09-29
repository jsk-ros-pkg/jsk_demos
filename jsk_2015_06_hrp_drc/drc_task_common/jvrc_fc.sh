#!/usr/bin/env bash

source `dirname ${0}`/fc_tmux_init.sh

tmux-newwindow executive "sleep 1; roslaunch drc_task_common fc_executive.launch fc:=false"
tmux-newwindow lookat "sleep 1; roslaunch drc_task_common lookat.launch"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common locomotion.launch INPUT_POINT_CLOUD:=/multisense/resize_1_2/points RUN_SELF_FILTER:=true SELF_FILTER_PARAM:=$(rospack find jaxon_ros_bridge)/config/jaxon_self_filter.yaml"
tmux-newwindow multisense_local "roslaunch jaxon_ros_bridge jaxon_multisense_local.launch"
tmux-newwindow multisense_remote "roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_multisense.launch"

tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
