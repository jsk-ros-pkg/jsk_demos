#!/usr/bin/env bash

source `dirname ${0}`/ocs_tmux_init.sh

tmux-newwindow roscore "roscore"
sleep 3
tmux-newwindow executive "roslaunch drc_task_common ocs_executive.launch"
tmux-newwindow ui "roslaunch drc_task_common ui.launch"
tmux-newwindow rviz "roslaunch drc_task_common ocs_rviz.launch ik_server_launch:=${ROBOT,,}-ik-server.launch"
tmux-newwindow misc "roslaunch drc_task_common ocs_misc.launch"
# tmux-newwindow vehicle "roslaunch drc_task_common vehicle_operator_station_main.launch USE_COM:=false ROBOT:=${ROBOT,,}"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common ocs_locomotion.launch"
tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common ocs_locomotion_planner.launch"
tmux-newwindow com "roslaunch drc_com_common operator_station_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux-newwindow lowspeed0 "rostopic echo /ocs_to_fc_low_speed/input"
tmux-newwindow lowspeed1 "rostopic echo /ocs_from_fc_low_speed/output"
tmux-newwindow gopos "rlwrap rosrun drc_task_common send-go-pos-command.l"
tmux send-keys -t ocs:tmp "exit" C-m
tmux a -t ocs
