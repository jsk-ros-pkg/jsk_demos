#!/usr/bin/env bash

tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t drc
        tmux send-keys -t drc:$1 "$2" C-m
    fi
}

if [ $# -gt 0 ]; then
    if [ $1 = "--kill" -o $1 = "k" ]; then
        tmux kill-session -t drc
        exit 0
    elif [ $1 = "--attach" -o $1 = "a" ]; then
        tmux a -t drc
        exit 0
    fi
fi

if `tmux has-session -t drc`; then
    echo -e "\e[1;33msession named drc already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named drc.\e[m"
    tmux new-session -d -s drc -n tmp
fi

tmux-newwindow object_detect "roslaunch drc_task_common object_detect.launch"
tmux-newwindow robot_motion "roslaunch drc_task_common robot_motion.launch"
tmux-newwindow robot_model "roslaunch drc_task_common robot_model.launch"
tmux-newwindow transformable_model "roslaunch drc_task_common transformable_model.launch"
tmux-newwindow teleop_device "roslaunch drc_task_common teleop_device.launch"
tmux-newwindow teleop_interface "roslaunch drc_task_common teleop_interface.launch"
tmux-newwindow remote_server "roslaunch drc_task_common remote_server.launch"
tmux-newwindow manipulation_processor "roslaunch drc_task_common manipulation_data_processor.launch"
tmux-newwindow rviz "rviz -d `rospack find drc_task_common`/config/drc_task_common.rviz"



tmux send-keys -t drc:tmp "exit" C-m
