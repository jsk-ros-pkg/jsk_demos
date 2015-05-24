#!/usr/bin/env bash

CMDNAME=$(basename $0)
FC_IP=localhost
OCS_IP=localhost

while getopts hf:o:ka OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") OCS_IP="$OPTARG";;
        "k") tmux kill-session -t ocs; exit;;
        "a") tmux a -t ocs; exit;;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o OCS_IP]"; exit;;
    esac
done

tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'  >/dev/null 2>&` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t ocs
        tmux send-keys -t ocs:$1 "$2" C-m
    fi
}

if `tmux has-session -t ocs`; then
    echo -e "\e[1;33msession named ocs already exists.\e[m"
    exit 1
else
    echo -e "\e[1;34mcreate new session named ocs.\e[m"
    tmux new-session -d -s ocs -n tmp
fi

tmux-newwindow executive "roslaunch drc_task_common ocs_executive.launch"
tmux-newwindow ui "roslaunch drc_task_common ui.launch"
tmux-newwindow rviz "roslaunch drc_task_common ocs_rviz.launch ik_server_launch:=${ROBOT,,}-ik-server.launch"
tmux-newwindow misc "roslaunch drc_task_common ocs_misc.launch"
tmux-newwindow vehicle "roslaunch drc_task_common vehicle_operator_station_main.launch USE_COM:=false ROBOT:=${ROBOT,,}"
tmux-newwindow com "roslaunch drc_com_common operator_station_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t ocs:tmp "exit" C-m
tmux a -t ocs
