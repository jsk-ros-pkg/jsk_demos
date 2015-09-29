#!/usr/bin/env bash

source `dirname ${0}`/fc_tmux_init.sh

tmux-newwindow executive "roslaunch drc_task_common fc_executive.launch fc:=false"
tmux-newwindow lookat "roslaunch drc_task_common lookat.launch"
tmux-newwindow locomotion "roslaunch drc_task_common locomotion.launch INPUT_POINT_CLOUD:=/multisense/resize_1_2/points"
tmux-newwindow multisense_local "roslaunch jaxon_ros_bridge/launch/jaxon_multisense_local.launch"
tmux-newwindow multisense_remote "roslaunch hrpsys_ros_bridge/launch/jaxon_jvrc_multisense.launch"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
