#!/usr/bin/env bash

source `dirname ${0}`/ocs_tmux_init.sh

tmux-newwindow executive "roslaunch drc_task_common ocs_executive.launch ocs:=false"
tmux-newwindow ui "roslaunch drc_task_common ui.launch INPUT_IMAGE:=/multisense/left/image_rect_color UI_FILE:=rqt_ui_jvrc.perspective"
tmux-newwindow rviz "roslaunch drc_task_common ocs_rviz.launch ik_server_launch:=${ROBOT,,}-ik-server.launch ocs:=false"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common ocs_locomotion.launch ocs:=false USE_SIMULATION:=true"
tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common ocs_locomotion_planner.launch ocs:=false USE_SIMULATION:=true"
tmux-newwindow plane_segmentation "rosparam load `rospack find drc_task_common`/config/jvrc_plane_param.yaml; roslaunch jsk_pcl_ros organized_multi_plane_segmentation.launch INPUT:=/multisense/resize_1_1/points"
tmux-newwindow dyn_param "rosrun dynamic_reconfigure dynparam set /euclidean_clustering min_size 20; rosrun dynamic_reconfigure dynparam set /euclidean_clustering tolerance 0.2 ; exit"
tmux-newwindow speedup_ri "rosrun roseus roseus $(rospack find hrpsys_ros_bridge_jvrc)/euslisp/walking_config.l \(main-loop\)"
tmux-newwindow qr_data_manager "rosrun drc_task_common qr-data-manager.l"
tmux-newwindow qr_data_reader "roslaunch drc_task_common zbar_ros_for_cameras.launch"
tmux-newwindow save_it ";rosrun pcl_ros pointcloud_to_pcd input:=/accumulated_heightmap_pointcloud/output"
tmux-newwindow okiagari "rosrun hrpsys_ros_bridge_jvrc jvrc-get-up.l"
tmux send-keys -t ocs:tmp "exit" C-m
tmux a -t ocs
