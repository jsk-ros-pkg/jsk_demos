#!/usr/bin/env bash

source `dirname ${0}`/fc_tmux_init.sh

if [ "$ROBOT" = "JAXON" -o "$ROBOT" = "JAXON_RED" ]; then
    tmux-newwindow multisense "sleep 1; roslaunch jaxon_ros_bridge jaxon_multisense_remote.launch"
fi
tmux-newwindow executive "sleep 1; roslaunch drc_task_common fc_executive.launch"
tmux-newwindow stereo_preprocess "sleep 1; roslaunch drc_task_common stereo_preprocess.launch"
tmux-newwindow laser_preprocess "sleep 1; roslaunch drc_task_common laser_preprocess.launch"
tmux-newwindow lookat "sleep 1; roslaunch drc_task_common lookat.launch"
tmux-newwindow valve_recognition "sleep 1; roslaunch drc_task_common valve_recognition.launch"
# tmux-newwindow drill_recognition "sleep 3; roslaunch drc_task_common drill_recognition.launch"
# tmux-newwindow drill_recognition_for_button "sleep 3; roslaunch drc_task_common drill_recognition_in_hand.launch"
# tmux-newwindow drill_recognition_for_wall "sleep 3; roslaunch drc_task_common drill_recognition_for_wall.launch"
# tmux-newwindow drill_recognition_for_put "sleep 3; roslaunch drc_task_common drill_recognition_for_put.launch"
# tmux-newwindow drill_button_checker "sleep 3; roslaunch drc_task_common drill_button_checker.launch"
tmux-newwindow door_recognition "sleep 5; roslaunch drc_task_common door_unvisible_handle_recognition.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common hose_connect.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common hose_grasp.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common debri_recognition.launch"
# tmux-newwindow panorama "roslaunch drc_task_common panorama.launch"
tmux-newwindow fisheye "sleep 5; roslaunch drc_task_common fisheye_lookat.launch"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common locomotion.launch"
# tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common locomotion_planner.launch"
tmux-newwindow vehicle "roslaunch drc_task_common vehicle_field_computer_main.launch USE_COM:=false ROBOT:=${ROBOT,,}"
tmux-newwindow misc "sleep 5; roslaunch drc_task_common fc_misc.launch"
tmux-newwindow com "sleep 5; roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
