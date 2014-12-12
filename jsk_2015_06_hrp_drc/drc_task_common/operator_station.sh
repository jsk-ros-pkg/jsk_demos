#!/usr/bin/env bash

tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t ocs
        tmux send-keys -t ocs:$1 "$2" C-m
    fi
}

if [ $# -gt 0 ]; then
    if [ $1 = "--kill" -o $1 = "k" ]; then
        tmux kill-session -t ocs
        exit 0
    elif [ $1 = "--attach" -o $1 = "a" ]; then
        tmux a -t ocs
        exit 0
    fi
fi

if `tmux has-session -t ocs`; then
    echo -e "\e[1;33msession named ocs already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named ocs.\e[m"
    tmux new-session -d -s ocs -n tmp
fi

tmux-newwindow task_ocs "roslaunch drc_task_common operator_station_main.launch"
tmux-newwindow robot_local "roslaunch jsk_hrp2_ros_bridge hrp2017_local.launch"
tmux-newwindow com_ocs "roslaunch drc_com_common operator_station_com.launch"
tmux-newwindow rviz "rviz -d `rospack find drc_task_common`/config/drc_task_common.rviz"
tmux send-keys -t ocs:tmp "exit" C-m
