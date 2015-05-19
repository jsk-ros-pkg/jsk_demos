#!/usr/bin/env bash

DRC_TASK_COMMON_LAUNCHES="ocs_executive.launch ui.launch ocs_rviz.launch ocs_misc.launch"
CMDNAME=$(basename $0)
FC_IP=localhost
OCS_IP=localhost
while getopts hf:o: OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") OCS_IP="$OPTARG";;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o OCS_IP]";;
    esac
done

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

for l in $DRC_TASK_COMMON_LAUNCHES
do
    tmux-newwindow $(basename $l .launch)_ocs "roslaunch drc_task_common $l"
done
tmux-newwindow operator_station_com_ocs "roslaunch drc_com_common operator_station_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t ocs:tmp "exit" C-m
